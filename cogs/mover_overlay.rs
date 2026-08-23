//! The overlay half of the decision tick: which windows this run will play, and
//! who is sampling the base while one of them is open.
//!
//! The layer itself — the window screen, the players, where the base stands — is
//! `cogs/overlay.rs`, and the composition is `reachy-clips`'. What lives here is
//! the join between them and the cog's slot: read the configured library once
//! per process rather than once per period, take the base over when a window
//! opens and hand it back when the last one closes, and turn each period into
//! the one setpoint the tick is asked for.
//!
//! Two rules carry the meaning, and neither is a safety rule:
//!
//! - **An overlay is presence.** Every refusal here — a window naming no
//!   motion, a library that will not establish, a composed setpoint the tick
//!   turned down — costs the run an overlay and nothing else. Safety is the
//!   envelope check and the per-tick step bound on whatever is commanded, and
//!   that is the tick's.
//! - **Every base plan is planned once, by the motion library.** A composed
//!   setpoint abandons any move the tick is running, so while a window is open
//!   this cog carries the base itself — and it carries it along the same path,
//!   with the antenna directions resolved and the clock floored, that commanding
//!   the move outright would have run.
//!
//! No clock is read. Every instant arrives as an argument, and nothing here
//! allocates.

use brenn_reachy__cogs__config_clk_rs::{ClipLibraryConfig, ClipLibraryConfigWire};
use brenn_reachy__cogs__mover_clk_rs::MoverStateWire;
use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
use brenn_reachy__motion__tick_state_clk_rs::MotionSnap;
use core::time::Duration;
use motion_slots::configured;
use overlay::{Base, Overlays, Windows, read_base, write_base};
use reachy_clips::compose::{compose, uncompose};
use reachy_clips::config::ValidatedLibrary;
use reachy_motion::joints::JointTargets;
use reachy_motion::tick::{
    ClockStretch, MotionCommand, MotionConfig, floor_move_clock, last_targets, plan_move,
};
use reachy_motion::traj::{MoveDurations, WarpKind};

use crate::MoverCounters;

/// How every base move this cog plans is shaped.
///
/// One shape for all of them, because they are one kind of thing: a transition
/// of the reference an overlay rides on, whether the tick is sampling it or this
/// cog is.
const WARP: WarpKind = WarpKind::MinJerk;

/// Where the base is being sent, and over what clocks.
#[derive(Clone, Copy)]
pub struct Goal {
    /// The configuration to reach.
    pub target: JointTargets,
    /// How long each mechanical group takes to get there, as configured. Only
    /// ever lengthened from here, never shortened.
    pub durations: MoveDurations,
}

/// What the overlay layer found this execution.
pub(crate) struct Screen<'a> {
    /// The windows this run will play, in the rows the schedule gave them.
    pub windows: Windows<'a>,
    /// Whether a composed setpoint carrying an overlay has been refused for the
    /// schedule now standing. While it holds, no window is taken up.
    pub latched: bool,
}

/// What one sample decided, and whether an overlay was riding it.
pub(crate) struct Commanded {
    /// The one command this sample asks the tick for, if it asks for any.
    pub command: Option<MotionCommand>,
    /// Whether the setpoint carried a live overlay contribution. What makes a
    /// refusal the layer's rather than the base's.
    pub overlaid: bool,
}

/// Where the last period left the commanded stream, and how close the machine
/// stood to the envelope's fences.
///
/// Read off the tick's state by the caller, which holds it validated for the
/// tick anyway: the take-up of the hottest schema in the tree is one per sample,
/// and a second one here to read two numbers would be paid at control rate.
#[derive(Clone, Copy)]
pub(crate) struct Anchor {
    /// The setpoint the last period commanded. What a handover starts the base
    /// at and what a re-anchor subtracts the vacated contribution from.
    pub setpoint: JointTargets,
    /// How close the measured pose stood to the nearest envelope fence.
    pub margin: f64,
}

impl Anchor {
    /// Read it off the tick's state.
    pub(crate) fn of(snap: &MotionSnap) -> Self {
        Self {
            setpoint: last_targets(snap),
            margin: snap.present_min_margin,
        }
    }
}

/// What the schedule asks of the base at one instant.
pub(crate) struct Ask {
    /// The sample's own grid instant, nanoseconds.
    pub now_ns: i64,
    /// The control period.
    pub period: Duration,
    /// The rate the per-tick step bounds are judged at, hertz.
    pub tick_hz: f64,
    /// A step the machine has not been sent to yet: a fresh posture, or the
    /// same one under a bumped epoch.
    pub fresh: Option<Goal>,
    /// Where the last dispatched step was sending the base, if one was. What a
    /// handover has to keep going toward: a composed setpoint abandons the
    /// tick's move, so the move has to be re-planned here or it is lost.
    pub standing: Option<Goal>,
}

/// Screen the schedule's overlay windows, and say whether the layer is latched.
///
/// Nothing is screened while the machine is disengaged: the windows would be
/// taken up by no sample.
///
/// Both answers are per schedule rather than per period. A schedule arriving
/// with an epoch the layer has not screened is a fresh set of windows: the latch
/// is cleared, and what the screen refuses is counted once, here, rather than
/// once for every control period the same windows stand for. The epoch is
/// recorded where the screening happens rather than on every wake, so a
/// disengaged wake does not spend the schedule's one screening on windows no
/// sample would have taken up.
///
/// A latched layer is not screened again either: nothing takes its windows up
/// until an epoch clears the latch, so building them would be structural work at
/// control rate for a result that is dropped.
///
/// The library is established through the state's own record of having walked
/// it: the configuration binding is immutable for the life of the process, so
/// the first execution that needs the library walks every frame and every later
/// one takes it up structurally. A library that will not establish refuses every
/// window the schedule carries — presence, never safety — and latches the layer
/// for that schedule, because a walk that failed once fails identically every
/// period and re-running it is the per-execution cost the recorded walk exists
/// to avoid.
///
/// # Panics
///
/// If the configured clip library does not read as one, which is a process built
/// against another schema rather than memory gone wrong.
pub(crate) fn screen<'a>(
    clips: &'a ClipLibraryConfigWire,
    state: &mut MoverStateWire,
    schedule: Option<&SessionScheduleWire>,
    engaged: bool,
    counters: &mut MoverCounters,
) -> Screen<'a> {
    let rows = schedule.map_or(0, |schedule| schedule.overlays().len());
    let (Some(schedule), true, 1..) = (schedule, engaged, rows) else {
        // Nothing to play, so nothing to establish. A schedule of postures alone
        // never touches the library -- and neither does one nobody is engaged on:
        // no sample takes a window up while the machine is disengaged, and
        // establishing the library for windows nothing will play would be
        // structural work at control rate for a result that is dropped. The
        // interval between a script being accepted and an engagement concluding
        // is seconds of aux traffic long, so that is not a rare wake.
        return Screen {
            windows: Windows::default(),
            latched: state.overlay_latch(),
        };
    };

    // A schedule this layer has already screened. Epoch zero is the one no
    // session publishes, so an unwritten slot reads as "nothing screened yet"
    // without a second flag saying so.
    let fresh = schedule.epoch() != state.latch_epoch();
    let mut latched = state.overlay_latch();
    if fresh {
        latched = false;
        state.set_overlay_latch(false);
        state.set_latch_epoch(schedule.epoch());
    }
    if latched {
        return Screen {
            windows: Windows::default(),
            latched,
        };
    }

    let library: &ClipLibraryConfig = configured(clips, "the mover's clip library");
    let walked = state.library_walked();
    let established = if walked {
        ValidatedLibrary::resumed(library)
    } else {
        ValidatedLibrary::of(library)
    };
    let Ok(established) = established else {
        // The whole library, so every window it could have named. Counted once
        // per schedule like any other refusal, and the base streams alone. The
        // layer is latched with it: the same library refuses the same way every
        // period, so a walk repeated at control rate would buy nothing and cost
        // the frame reads.
        if fresh {
            counters.overlays_refused += rows as u64;
        }
        state.set_overlay_latch(true);
        return Screen {
            windows: Windows::default(),
            latched: true,
        };
    };
    if !walked {
        state.set_library_walked(true);
    }

    let windows = Windows::of(schedule, &established);
    if fresh {
        counters.overlays_refused += windows.refused();
    }
    Screen { windows, latched }
}

/// Decide the one command this sample asks the tick for.
///
/// Three answers, and which one it is turns on whether a window covers this
/// instant:
///
/// - **No window, and the tick has the base.** The ordinary posture path: the
///   step the schedule asks for, on a clock long enough to carry the span it
///   actually covers.
/// - **A window covers it.** This cog has the base. It takes it over from the
///   setpoint the last period commanded, samples it, composes the players'
///   weighted deltas onto it and asks for the result as one tracked setpoint,
///   which faces the envelope check and the step bound like any other command.
/// - **No window, and this cog has the base.** The hand-back: the rows are
///   vacated, the ownership record is cleared, and the base is re-planned from
///   the composed setpoint that was last commanded toward where the schedule is
///   sending it. The contribution the closing window was carrying is absorbed
///   into that plan's starting point and decays under the same step bound as
///   every other move, so the commanded stream is continuous across the close.
///
/// `anchor` is where the last period left the stream, read off the tick's state
/// by the caller: this function holds the slot open for the base and the players,
/// which cannot be borrowed beside the state inside it.
pub(crate) fn decide(
    cfg: &MotionConfig,
    state: &mut MoverStateWire,
    windows: &Windows<'_>,
    latched: bool,
    ask: &Ask,
    anchor: &Anchor,
    counters: &mut MoverCounters,
) -> Commanded {
    let Anchor { setpoint, margin } = *anchor;
    // Reused memory holding whatever an older run left: a record that is not a
    // base is counted and cleared, which leaves the tick owning the base, and
    // the next window opening takes it over afresh.
    let held = match read_base(state.base()) {
        Ok(held) => held,
        Err(_) => {
            counters.refused_base += 1;
            write_base(state.base_mut(), None);
            None
        }
    };

    if latched || !windows.any_covers(ask.now_ns) {
        let Some(base) = held else {
            // The tick owns the base and keeps it, and the step the schedule
            // asks for is floored here: the tick's own step guard is documented
            // to expect a clock that was right-sized before it was commanded,
            // and this is the caller that commands it. A posture move sweeps
            // both antennas between the stow and the working posture, mirrored,
            // so what the floor mostly does here is part the pair at their
            // crossing.
            return Commanded {
                command: ask
                    .fresh
                    .map(|goal| planned(cfg, &setpoint, goal, ask.tick_hz, counters)),
                overlaid: false,
            };
        };
        release(state, ask.now_ns);
        // The hand-back, which is the re-anchor with nothing left riding: the
        // tick plans from the setpoint the last period commanded, so the
        // contribution the closing window was carrying is absorbed into the
        // plan's own starting point. Where the base was going, or where it
        // stands when nothing is sending it anywhere -- and in that last case
        // over one period, the shortest clock there is, which the floor
        // lengthens to whatever the offset being absorbed actually needs.
        let goal = ask.fresh.or(ask.standing).unwrap_or(Goal {
            target: base.targets,
            durations: MoveDurations::uniform(ask.period),
        });
        return Commanded {
            command: Some(planned(cfg, &setpoint, goal, ask.tick_hz, counters)),
            overlaid: false,
        };
    }

    // The handover: the base starts at the setpoint the last period commanded,
    // because the tick's own move is over the moment a tracked setpoint arrives.
    let handover = held.is_none();
    let mut base = held.unwrap_or_else(|| Base::held(setpoint));

    // Where the base is headed when nothing new asks for anything: the end of
    // the move it is on, over that move's own clocks, or where it stands. What a
    // re-anchored base decays back toward.
    let carrying_on = Goal {
        target: base
            .path
            .as_ref()
            .map_or(base.targets, |run| *run.path.target()),
        durations: base
            .path
            .as_ref()
            .map_or(MoveDurations::uniform(ask.period), |run| {
                run.path.durations()
            }),
    };

    // The rows this period, and whether a window closed out from under one. A
    // player that ran out is not that: its own exit ramp took its contribution
    // to zero, which is the whole point of the ramp.
    let playing = active_rows(state);
    let (samples, refused) = {
        let (mut layer, refusals) =
            Overlays::take_up(&mut state.players_mut()[..], windows, ask.now_ns);
        (layer.sample(ask.period), refusals.players)
    };
    counters.players_refused += refused;
    let vacated = playing & !active_rows(state) != 0;

    // The re-anchor, which is the mechanism the whole layer's continuity rests
    // on: the base is moved to the composed setpoint that was last commanded
    // less what still rides it, so this period's composition comes out at the
    // same setpoint as the last one, and the vacated contribution decays from
    // there as a planned, step-bounded move like any other.
    if vacated {
        base = Base::held(uncompose(setpoint, samples.as_slice()));
    }
    // A handover or a re-anchor is sent on toward wherever the schedule was
    // already sending the base; a period that merely carries on is not
    // re-planned at all, and samples the move it is already on.
    let resumes = (handover || vacated).then(|| ask.standing.unwrap_or(carrying_on));
    if let Some(goal) = ask.fresh.or(resumes) {
        steer(cfg, &mut base, goal, ask.tick_hz, margin, counters);
    }

    let targets = base.step(ask.period);
    write_base(state.base_mut(), Some(&base));
    Commanded {
        overlaid: !samples.is_empty(),
        command: Some(MotionCommand::Track(compose(targets, samples.as_slice()))),
    }
}

/// Which overlay rows hold a player at all, as a bit per row.
///
/// Read either side of a take-up, which is what says a window closed out from
/// under a player rather than a player having played itself out: a row a window
/// no longer covers is emptied, and a row whose player ran out keeps it.
fn active_rows(state: &MoverStateWire) -> u8 {
    state
        .players()
        .iter()
        .enumerate()
        .filter(|(_, row)| row.active())
        .fold(0u8, |set, (row, _)| set | 1 << row)
}

/// Give the base back to the tick: no players, no ownership record.
///
/// What a disengagement, a fresh arming and the last window closing all do. A
/// player left behind would be picked up by whatever window next took its row,
/// and an ownership record left behind would have the next open window carry on
/// from a base belonging to an engagement that ended.
pub(crate) fn release(state: &mut MoverStateWire, now_ns: i64) {
    let (layer, _) = Overlays::take_up(&mut state.players_mut()[..], &Windows::default(), now_ns);
    debug_assert!(!layer.any(), "no window plays where there are no windows");
    write_base(state.base_mut(), None);
}

/// The move to `goal`, on a clock long enough to carry the span it covers.
///
/// A duration is configuration, sized for the spans an ordinary command covers;
/// where the machine physically stands is not, and a fixed clock over a span it
/// was never sized for steps past the per-tick guard partway through and
/// abandons the move. The pair's phase is the second thing a clock has to carry,
/// and the same pass asks for it. Both are reported rather than silent, in the
/// two totals `count_adjustment` tells apart: a span being lengthened is
/// configuration that no longer describes the move, and a pair being parted is
/// the geometry doing what it is there for.
fn planned(
    cfg: &MotionConfig,
    start: &JointTargets,
    goal: Goal,
    tick_hz: f64,
    counters: &mut MoverCounters,
) -> MotionCommand {
    let asked = asked_move(goal);
    let (floored, stretch) = floor_move_clock(cfg, start, &asked, tick_hz);
    count_adjustment(stretch, counters);
    floored
}

/// The move to `goal` as this cog asks for it, before the floor.
fn asked_move(goal: Goal) -> MotionCommand {
    MotionCommand::MoveTo {
        target: goal.target,
        durations: goal.durations,
        warp: WARP,
    }
}

/// The clocks the move to `goal` from `start` actually runs on.
///
/// A command the floor declines to measure comes back on the clocks it was
/// asked for.
#[must_use]
pub fn floored_clocks(
    cfg: &MotionConfig,
    start: &JointTargets,
    goal: Goal,
    tick_hz: f64,
) -> MoveDurations {
    match floor_move_clock(cfg, start, &asked_move(goal), tick_hz).0 {
        MotionCommand::MoveTo { durations, .. } => durations,
        _ => goal.durations,
    }
}

/// Count what the library did to a plan's clocks, in the one of the two totals
/// that says what it means.
///
/// A de-phasing is what the pair's own geometry asks for on every move that
/// sweeps both antennas between the stow and the working posture, so counting
/// it beside the anomalies would leave `base_stretched` climbing on a perfectly
/// healthy machine and saying nothing. The anomalous bucket takes the combined
/// case: a clock lengthened for its span and then de-phased is a span anomaly
/// whatever else happened to it.
fn count_adjustment(stretch: Option<ClockStretch>, counters: &mut MoverCounters) {
    let Some(stretch) = stretch else {
        return;
    };
    let unmet = stretch
        .separation
        .is_some_and(|pair| !pair.met(stretch.separation_required));
    if stretch.span_stretched || unmet {
        counters.base_stretched += 1;
    } else {
        counters.base_dephased += 1;
    }
}

/// Send a base this cog is sampling toward `goal`.
///
/// Planned through the motion library's own planner, so the antenna directions
/// are resolved, the clock is floored and the envelope is judged exactly as
/// commanding the move outright would have judged them. A plan the machine will
/// not run is counted and changes nothing: the base carries on along whatever it
/// was already on, which per the fault doctrine is a refused plan and not a
/// fault.
fn steer(
    cfg: &MotionConfig,
    base: &mut Base,
    goal: Goal,
    tick_hz: f64,
    margin: f64,
    counters: &mut MoverCounters,
) {
    match plan_move(
        cfg,
        &base.targets,
        &goal.target,
        goal.durations,
        WARP,
        tick_hz,
        Some(margin),
    ) {
        Ok((path, stretch)) => {
            count_adjustment(stretch, counters);
            base.retarget(&path);
        }
        Err(_) => counters.refused_base += 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use reachy_motion::joints::JointRef;
    use reachy_motion::phase::PhaseSeparation;
    use reachy_motion::postures::{neutral_targets, stow_pose_targets};

    /// The separation a pair is held to here.
    const SEPARATION: f64 = 0.6;

    /// The cycle rate every clock below is floored against.
    const TICK_HZ: f64 = 50.0;

    /// A measurement of a pair standing `offset` from mirrored.
    fn parted(offset: f64) -> PhaseSeparation {
        PhaseSeparation {
            offset,
            later: JointRef::AntennaRight,
            at: Duration::from_millis(500),
            leader_rate: 5.0,
        }
    }

    /// A report of what the pass did, over one pair of clocks.
    fn stretch(
        span_stretched: bool,
        dephased: bool,
        separation: Option<PhaseSeparation>,
    ) -> ClockStretch {
        let requested = MoveDurations::uniform(Duration::from_millis(800));
        ClockStretch {
            requested,
            effective: if span_stretched || dephased {
                MoveDurations {
                    head: Duration::from_millis(800),
                    antennas: [Duration::from_millis(970), Duration::from_millis(800)],
                }
            } else {
                requested
            },
            separation,
            separation_required: SEPARATION,
            dephased,
            span_stretched,
        }
    }

    /// Which of the two totals a report lands in.
    fn counted(stretch: Option<ClockStretch>) -> (u64, u64) {
        let mut counters = MoverCounters::default();
        count_adjustment(stretch, &mut counters);
        (counters.base_stretched, counters.base_dephased)
    }

    /// Every shape of report the library produces is counted, in exactly one of
    /// the two totals, and the anomalous total takes every shape but one.
    ///
    /// The arm the cog cannot reach from a scenario is the one this is here for:
    /// a pair no clock could part reports neither flag, and it is the single
    /// condition an operator most needs `base_stretched` to be nonzero for -- a
    /// pair commanded straight through the band its de-phasing exists to clear.
    /// A regression routing it to the routine total would hide it inside a
    /// figure that is expected to climb.
    #[test]
    fn a_pair_nothing_could_part_is_an_anomaly_and_a_parted_one_is_not() {
        assert_eq!(counted(None), (0, 0), "nothing to say is nothing to count");
        assert_eq!(
            counted(Some(stretch(false, true, Some(parted(0.61))))),
            (0, 1),
            "a pair parted at its crossing is the geometry working"
        );
        assert_eq!(
            counted(Some(stretch(true, false, None))),
            (1, 0),
            "a clock that could not carry its own span is news"
        );
        assert_eq!(
            counted(Some(stretch(true, true, Some(parted(0.61))))),
            (1, 0),
            "a span stretched and then parted is still a span anomaly"
        );
        assert_eq!(
            counted(Some(stretch(false, false, Some(parted(0.09))))),
            (1, 0),
            "a pair the pass could not part is the anomaly this total is for"
        );
    }

    /// The clocks a scenario derives are the floored ones, not the asked ones.
    ///
    /// The harness's whole claim is that its arithmetic and this cog's shaping
    /// are one number, and it reads that number out of here. A move the floor
    /// lengthens comes back longer; a move it leaves alone comes back exactly as
    /// asked, which is what says the answer is the floor's own and not the
    /// fallback for a command shape this function does not recognise.
    #[test]
    fn the_clocks_a_caller_reads_back_are_the_floored_ones() {
        let cfg = reachy_motion::tick::default_motion_config();
        let from = stow_pose_targets();
        let to = neutral_targets();

        let asked = MoveDurations::uniform(Duration::from_millis(20));
        let floored = floored_clocks(
            cfg,
            &from,
            Goal {
                target: to,
                durations: asked,
            },
            TICK_HZ,
        );
        assert!(
            floored.head > asked.head,
            "a whole stand-up in one period runs on a longer clock: {floored:?}"
        );

        let roomy = MoveDurations::uniform(Duration::from_secs(8));
        let unchanged = floored_clocks(
            cfg,
            &from,
            Goal {
                target: to,
                durations: roomy,
            },
            TICK_HZ,
        );
        assert_eq!(
            unchanged.head, roomy.head,
            "a head clock with room to spare is the clock that was asked for"
        );
    }
}
