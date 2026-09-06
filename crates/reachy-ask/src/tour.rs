//! The library tour: one script per motion, in library order, at recorded
//! pace.
//!
//! The gesture beside this one asks the machine to raise and fold, and the
//! analyzer reads that one story. The tour asks for every motion the deployed
//! library holds, one after another, and the record it leaves is the plant's
//! own answer to content nobody has played on this machine before: what the
//! servos did with each commanded track, how far behind they ran, how near the
//! antenna tips came to meeting. That record is the point of the run. Nothing
//! here judges it — `cogs/library_tour_report` does — and nothing here decides
//! which motions exist: the sidecar the box loads is the plan.
//!
//! The plan is a list of scripts and the sender's own clock for them. Each
//! script raises or keeps the base, plays one motion at 1.0x, and ends with a
//! stow it will normally never reach, because the next script replaces it
//! first; the stow is what folds the machine if the next script never comes.
//! The sender sends script `k + 1` once script `k`'s play window has closed by
//! its own clock. That clock is the sender's and not the session's: the window
//! opens at the wake that took the script, so a replacement lands up to one
//! session period either side of the window's real close, and the tail of a
//! blend-out it lands inside of is absorbed by the mover's hand-back rather
//! than stepped.
//!
//! The run ends itself. A tour long enough for a whole library is too long to
//! sit under a fixed budget that would then be spent idling, and a run that has
//! to be watched to be ended is not an unattended run — so when the story is
//! over the sender asks the launcher to quit, and the harness's `timeout` is
//! only the backstop. [`budget`] is that backstop, computed from the same plan
//! the sender runs, so the two cannot disagree.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use motion_proto::{MotionScript, Play, Posture, Step};
use reachy_edge::{MotionEntry, MotionTable, POLL, STOW_DURATION_MS};

use crate::gesture::UP_AFTER_MS;

/// How long the move to the upright posture is given, milliseconds.
///
/// The mover's own `up_duration_ns`, restated here because the sender does not
/// link the mover's parameters. The first motion cannot start until the raise
/// has finished, so this is a term of the first script's play offset; a case
/// below pins it against the number the scenarios read out of the textproto.
pub const UP_DURATION_MS: u64 = 800;

/// The gap between a script's base step settling and its motion starting,
/// milliseconds.
///
/// It is also the gap between one motion's window closing and the next one's
/// first frame, because the next script's play sits this far past its own
/// receipt: long enough for the base to have taken itself back after the
/// previous overlay faded, short enough that a library's worth of them is not
/// most of the run.
pub const PLAY_AFTER_MS: u64 = 1000;

/// How long after a motion's window closes the script's own stow would open,
/// milliseconds.
///
/// Slack, not a plan: the next script normally replaces this one before the
/// stow comes due. It covers the sender's own process scheduling on the unit,
/// and it is what folds the machine if the sender stops saying anything.
pub const STOW_MARGIN_MS: u64 = 5000;

/// How long the session is allowed to take releasing after its last stow,
/// milliseconds.
///
/// The release's own arithmetic — the stow dwell, then two sweeps of the bus —
/// is `scenario::release_allowance_cycles`, and a case below pins this number
/// against it. Written out rather than derived because the sender links no
/// scenario harness, and it is the clock ending for a machine that parked, so
/// a figure short of the release would end the tour before the row it waits
/// for.
pub const RELEASE_ALLOWANCE_MS: u64 = 4000;

/// The sender's own margin on the clock ending, milliseconds.
///
/// The clock ending is the one for a machine that parked mid-tour and will
/// never narrate a release; the margin is there so it is not raced against the
/// release row on a healthy run.
pub const END_MARGIN_MS: u64 = 5000;

/// How long the machine is allowed to take commissioning, for the backstop
/// budget only, milliseconds.
///
/// The survey's own arithmetic is `scenario::commission_allowance_cycles`, and
/// a case below pins this number against it: the budget is the only thing
/// between a tour and a launcher nobody stops, and a budget tighter than the
/// run is a red `124` on a tour that would have finished. Written out rather
/// than derived because the sender links no scenario harness.
pub const COMMISSIONING_ALLOWANCE_MS: u64 = 6000;

/// How late one leg may go out against the plan's clock, for the backstop
/// budget only.
///
/// The sender dispatches at the top of its loop and every iteration blocks in a
/// read on the narration port for up to one [`POLL`], so a leg comes due while
/// the loop is asleep and goes out when the read returns. The next leg's clock
/// runs from the send that actually happened — which is right, because the
/// daemon's window opened late too — so the lateness accumulates rather than
/// cancelling, and the budget carries one of these per leg.
pub const DISPATCH_ALLOWANCE: Duration = POLL;

/// What the backstop budget adds to the plan's own end, seconds.
///
/// The budget is not the stop — the sender is — so it is generous rather than
/// tight: a tour that reaches it has already failed, and thirty seconds keeps a
/// slow unit from failing a run that would otherwise have finished.
pub const BUDGET_MARGIN_S: u64 = 30;

/// Where the launcher serves its control API.
///
/// Fixed: nothing on the unit passes it another port, and the API cannot be
/// configured off. The sender assumes the launcher is listening here and that
/// `POST /quit` triggers its shutdown sequence.
pub const LAUNCHER_CONTROL: &str = "127.0.0.1:8080";

/// How long the quit request waits, at connect and at write.
const QUIT_TIMEOUT: Duration = Duration::from_secs(5);

/// How long the quit request waits for the launcher's answer.
///
/// The launcher completes its shutdown before responding, so the answer may
/// take several seconds to arrive. A timeout shorter than the shutdown turns a
/// healthy quit into a red run, so this is past the expected sequence with room
/// to spare.
const QUIT_READ_TIMEOUT: Duration = Duration::from_secs(20);

/// How long the quit keeps offering itself to a launcher that is not listening
/// yet.
///
/// The shell starts the sender before the launcher, so an ending the sender
/// reaches in its own first milliseconds — a sidecar that will not read, a
/// narration port already held — arrives while nothing is bound to the control
/// port. Without this window the quit is refused, the launcher then runs the
/// whole backstop with a machine nobody asked to move, and the cheapest failure
/// to diagnose becomes the most expensive to observe. Fifteen seconds is room
/// for the launcher's own start-up; a launcher that never binds at all is a
/// different red and is still reached.
pub const QUIT_CONNECT_WINDOW: Duration = Duration::from_secs(15);

/// How long the quit waits between offers while the control port is not bound.
const QUIT_RETRY_PAUSE: Duration = Duration::from_millis(100);

/// One motion's script and the sender's clock for it.
#[derive(Clone, Debug, PartialEq)]
pub struct Leg {
    /// The motion this leg plays.
    pub name: String,
    /// The index the sidecar gave it, which is the order the tour runs in.
    pub motion_id: u16,
    /// What goes out on the wire.
    pub script: MotionScript,
    /// Milliseconds after this script goes out at which its play window closes,
    /// which is when the next one goes out.
    pub window_close_ms: u64,
    /// Milliseconds after this script goes out at which its own stow would have
    /// finished, had nothing replaced it. The last leg's is the tour's end.
    pub stow_end_ms: u64,
}

/// The whole plan: one leg per motion, in library order.
#[derive(Clone, Debug, PartialEq)]
pub struct Tour {
    legs: Vec<Leg>,
}

impl Tour {
    /// The tour `table` describes: every motion it holds, ordered by the index
    /// the box invokes it under.
    ///
    /// # Errors
    ///
    /// A table with no motions in it — there is no tour to run and a green exit
    /// over a log of nothing is the failure this catches — or a leg whose
    /// timeline the wire contract refuses, which needs a motion longer than any
    /// clip the library can hold.
    pub fn of(table: &MotionTable) -> Result<Self, String> {
        let mut rows: Vec<(&str, &MotionEntry)> = table.entries().collect();
        rows.sort_by_key(|(_, entry)| entry.motion_id);
        if rows.is_empty() {
            return Err("the names sidecar holds no motions; there is no tour to run".to_owned());
        }
        let mut legs = Vec::with_capacity(rows.len());
        for (index, (name, entry)) in rows.into_iter().enumerate() {
            legs.push(leg(index, name, entry)?);
        }
        Ok(Self { legs })
    }

    /// The legs, in the order they go out.
    #[must_use]
    pub fn legs(&self) -> &[Leg] {
        &self.legs
    }

    /// A tour of `legs` exactly as given.
    ///
    /// For a case driving the send loop, which needs a plan whose clock runs in
    /// milliseconds rather than the minutes a real library's does.
    #[cfg(test)]
    pub fn of_legs(legs: Vec<Leg>) -> Self {
        Self { legs }
    }

    /// How long the whole tour takes, from the launcher's start to the sender's
    /// last word, with the allowances the harness states.
    ///
    /// This is the backstop the harness wraps the launcher in, not the stop: a
    /// tour that reaches it is a red run. It comes from the same legs the
    /// sender runs on, so the plan and the budget cannot disagree.
    ///
    /// The plan's own clock is not the whole of it. The sender dispatches a leg
    /// at the top of a loop iteration and each iteration waits out a read on
    /// the narration port, so a leg goes out up to one poll late and the next
    /// leg's clock starts from when it actually went — the lateness accumulates
    /// over the library. [`DISPATCH_ALLOWANCE`] per leg is that term, without
    /// which the budget is short of the run by a poll per motion.
    #[must_use]
    pub fn budget(&self) -> Duration {
        let last = self.legs.last().expect("a tour holds at least one leg");
        let sends: u64 = self
            .legs
            .iter()
            .take(self.legs.len() - 1)
            .map(|leg| leg.window_close_ms)
            .sum();
        let dispatch = DISPATCH_ALLOWANCE
            .as_millis()
            .try_into()
            .unwrap_or(u64::MAX)
            .saturating_mul(self.legs.len() as u64);
        let plan_ms = COMMISSIONING_ALLOWANCE_MS
            + sends
            + dispatch
            + last.stow_end_ms
            + RELEASE_ALLOWANCE_MS
            + END_MARGIN_MS;
        Duration::from_secs(plan_ms.div_ceil(1000) + BUDGET_MARGIN_S)
    }
}

/// The `index`-th leg of the tour.
///
/// The first leg raises the machine, because the tour starts at rest and the
/// base has to be somewhere before a delta rides on it; the edge cannot date a
/// request forward, so the arming lead lives in the offset exactly as the wake
/// gesture's does, and the first motion waits out the raise on top of that.
/// Every later leg says `keep`: the machine is already up, and restating `up`
/// would retarget the base mid-hand-back.
fn leg(index: usize, name: &str, entry: &MotionEntry) -> Result<Leg, String> {
    let seq = index as u64 + 1;
    let (base, play_after) = if index == 0 {
        (
            Step::new(UP_AFTER_MS, Posture::Up),
            UP_AFTER_MS + UP_DURATION_MS + PLAY_AFTER_MS,
        )
    } else {
        (Step::keep(0), PLAY_AFTER_MS)
    };
    let window_close_ms = play_after + entry.window.span_ms(1.0);
    let stow_after = window_close_ms + STOW_MARGIN_MS;
    let stow_end_ms = stow_after + u64::from(STOW_DURATION_MS);
    let script = MotionScript::new(
        crate::gesture::ASK_POD,
        seq,
        vec![
            base,
            Step::play(play_after, Play::new(name)),
            Step::new(stow_after, Posture::Stow),
        ],
        stow_end_ms,
    )
    .map_err(|error| format!("the tour's script for `{name}` is not a lawful timeline: {error}"))?;
    Ok(Leg {
        name: name.to_owned(),
        motion_id: entry.motion_id,
        script,
        window_close_ms,
        stow_end_ms,
    })
}

/// Ask the launcher to quit, and wait for it to say it will.
///
/// One request, no HTTP client: three header lines and an empty body, and the
/// reply read until the launcher closes the socket. What comes back matters —
/// a refused connection or a status that is not 2xx means the launcher is
/// still running and the run is about to ride its backstop, which is a red of
/// its own whether the ending it was quitting from was green or red.
///
/// A connection refused is not that answer yet, for `window`: the sender starts
/// before the launcher does, so an ending reached in the sender's own first
/// moments finds the control port unbound and has to wait for it rather than
/// leaving the run to the backstop. `stop` cuts that wait short: a stop signal
/// says the launcher has already gone, so there is nothing left to offer the
/// quit to and the shell is waiting on this process to say what happened.
/// Nothing else is retried — a launcher that took the connection and then
/// refused the request has answered.
///
/// # Errors
///
/// The connection, the write, the read, a stop signal while the control port
/// was not listening, or a status line that is not 2xx.
pub fn quit_launcher(
    control: SocketAddr,
    window: Duration,
    stop: &AtomicBool,
) -> Result<(), String> {
    let mut stream = connect(control, window, stop)?;
    stream
        .set_write_timeout(Some(QUIT_TIMEOUT))
        .and_then(|()| stream.set_read_timeout(Some(QUIT_READ_TIMEOUT)))
        .map_err(|error| format!("setting the timeouts on the launcher's control API: {error}"))?;
    let request = format!("POST /quit HTTP/1.1\r\nHost: {control}\r\nConnection: close\r\n\r\n");
    stream
        .write_all(request.as_bytes())
        .map_err(|error| format!("asking the launcher to quit: {error}"))?;
    let mut reply = String::new();
    stream
        .read_to_string(&mut reply)
        .map_err(|error| format!("reading the launcher's answer to the quit: {error}"))?;
    let status = reply.lines().next().unwrap_or("").trim().to_owned();
    if status.split_whitespace().nth(1).is_some_and(is_success) {
        return Ok(());
    }
    Err(format!(
        "the launcher did not accept the quit: it answered `{status}`"
    ))
}

/// Reach the control port, waiting out a launcher that has not bound it yet.
fn connect(control: SocketAddr, window: Duration, stop: &AtomicBool) -> Result<TcpStream, String> {
    let until = Instant::now() + window;
    loop {
        if stop.load(Ordering::Relaxed) {
            return Err(format!(
                "the stop arrived while nothing was listening at {control}: the launcher had \
                 already gone, so there was nothing to ask to quit"
            ));
        }
        match TcpStream::connect_timeout(&control, QUIT_TIMEOUT) {
            Ok(stream) => return Ok(stream),
            Err(error) => {
                if error.kind() != std::io::ErrorKind::ConnectionRefused || Instant::now() >= until
                {
                    return Err(format!(
                        "connecting to the launcher's control API at {control}: {error}"
                    ));
                }
                std::thread::sleep(QUIT_RETRY_PAUSE);
            }
        }
    }
}

/// Whether an HTTP status code is a success.
fn is_success(code: &str) -> bool {
    code.parse::<u16>()
        .is_ok_and(|code| (200..300).contains(&code))
}

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};
    use std::net::{Ipv4Addr, SocketAddr, TcpListener};
    use std::sync::atomic::AtomicBool;
    use std::thread;
    use std::time::Duration;

    use motion_proto::{Base, PlayWindow, Posture};
    use reachy_edge::{Edge, EdgeConfig, MotionEntry, MotionTable, STOW_DURATION_MS};

    use scenario::{
        PERIOD_NS, commission_allowance_cycles, engage_allowance_cycles, release_allowance_cycles,
        up_clocks,
    };

    use super::{
        BUDGET_MARGIN_S, COMMISSIONING_ALLOWANCE_MS, DISPATCH_ALLOWANCE, END_MARGIN_MS,
        LAUNCHER_CONTROL, PLAY_AFTER_MS, RELEASE_ALLOWANCE_MS, STOW_MARGIN_MS, Tour, UP_AFTER_MS,
        UP_DURATION_MS, quit_launcher,
    };
    use crate::gesture::ASK_POD;

    /// A round instant, so a stamp read off the wrong side of the edge shows.
    const ARRIVAL_NS: i64 = 1_700_000_000_000_000_000;

    /// A table of `names`, numbered in the order given, each with its own
    /// window so a leg that read the wrong row shows as a wrong offset.
    fn table(names: &[(&str, u64, u64)]) -> MotionTable {
        MotionTable::of(
            names
                .iter()
                .enumerate()
                .map(|(index, (name, duration, blend))| {
                    (
                        (*name).to_owned(),
                        MotionEntry {
                            motion_id: u16::try_from(index).expect("a small fixture"),
                            window: PlayWindow {
                                duration_ms: *duration,
                                blend_out_ms: *blend,
                            },
                        },
                    )
                }),
        )
    }

    /// The two-motion fixture the plan cases read.
    fn two() -> MotionTable {
        table(&[
            ("pollen/dances/nod", 4000, 200),
            ("pollen/emotions/oops", 2500, 120),
        ])
    }

    #[test]
    fn the_tour_runs_the_library_in_index_order_one_motion_per_script() {
        let tour = Tour::of(&two()).expect("a two-motion library");
        let names: Vec<&str> = tour.legs().iter().map(|leg| leg.name.as_str()).collect();
        assert_eq!(names, vec!["pollen/dances/nod", "pollen/emotions/oops"]);
        let seqs: Vec<u64> = tour.legs().iter().map(|leg| leg.script.seq()).collect();
        assert_eq!(seqs, vec![1, 2], "a replacement outranks what it replaces");

        for leg in tour.legs() {
            assert_eq!(leg.script.pod(), ASK_POD);
            let offsets: Vec<u64> = leg
                .script
                .steps()
                .iter()
                .map(|step| step.after_ms)
                .collect();
            let mut ascending = offsets.clone();
            ascending.sort_unstable();
            assert_eq!(offsets, ascending, "a timeline the wire contract admits");
            assert_eq!(leg.script.steps().len(), 3, "a base, a play and a stow");
            assert_eq!(
                leg.script.steps()[2].action.base().and_then(Base::posture),
                Some(Posture::Stow),
                "every script ends the session stowed if the next one never comes",
            );
            assert_eq!(
                leg.script.timeout_ms(),
                leg.stow_end_ms,
                "the timeout falls exactly on the end of the closing stow",
            );
        }
    }

    #[test]
    fn the_first_leg_raises_and_every_later_one_keeps_the_base() {
        let tour = Tour::of(&two()).expect("a two-motion library");
        let first = &tour.legs()[0];
        assert_eq!(
            first.script.steps()[0].action.base(),
            Some(Base::Posture(Posture::Up)),
            "the tour starts at rest, and a delta has to ride on something",
        );
        assert_eq!(first.script.steps()[0].after_ms, UP_AFTER_MS);
        assert_eq!(
            first.script.steps()[1].after_ms,
            UP_AFTER_MS + UP_DURATION_MS + PLAY_AFTER_MS,
        );

        let later = &tour.legs()[1];
        assert_eq!(
            later.script.steps()[0].action.base(),
            Some(Base::Keep),
            "restating `up` would retarget the base mid-hand-back",
        );
        assert_eq!(later.script.steps()[0].after_ms, 0);
        assert_eq!(later.script.steps()[1].after_ms, PLAY_AFTER_MS);
    }

    #[test]
    fn a_legs_clock_is_its_own_window_and_the_slack_before_its_stow() {
        let tour = Tour::of(&two()).expect("a two-motion library");
        let first = &tour.legs()[0];
        // The fixture's first motion: 4000 ms at 1.0x plus a 200 ms fade.
        let span = 4200;
        assert_eq!(
            first.window_close_ms,
            UP_AFTER_MS + UP_DURATION_MS + PLAY_AFTER_MS + span,
        );
        assert_eq!(
            first.script.steps()[2].after_ms,
            first.window_close_ms + STOW_MARGIN_MS,
            "the stow opens a margin past the window, and the replacement lands first",
        );
        assert_eq!(
            first.stow_end_ms,
            first.window_close_ms + STOW_MARGIN_MS + u64::from(STOW_DURATION_MS),
        );
    }

    #[test]
    fn the_up_move_is_the_movers_own_clock() {
        assert_eq!(
            i64::try_from(UP_DURATION_MS).expect("a count of ms") * 1_000_000,
            scenario::UP_DURATION_NS,
            "the first motion waits out the raise, and the raise runs on the number the \
             mover's parameters state; the scenarios pin that number against the textproto",
        );
    }

    #[test]
    fn the_release_allowance_covers_the_release_the_scenarios_derive() {
        let allowance_ns = i64::try_from(RELEASE_ALLOWANCE_MS).expect("a count of ms") * 1_000_000;
        let derived_ns = release_allowance_cycles() * PERIOD_NS;
        assert!(
            allowance_ns >= derived_ns,
            "the clock ending allows {} ms for the release and the session's own arithmetic \
             wants {} ms; a tour whose plan is shorter than the machine's release ends before \
             the row it is waiting for",
            allowance_ns / 1_000_000,
            derived_ns / 1_000_000,
        );
    }

    #[test]
    fn the_commissioning_allowance_covers_the_survey_the_scenarios_derive() {
        let allowance_ns =
            i64::try_from(COMMISSIONING_ALLOWANCE_MS).expect("a count of ms") * 1_000_000;
        let derived_ns = commission_allowance_cycles() * PERIOD_NS;
        assert!(
            allowance_ns >= derived_ns,
            "the backstop budget allows {} ms for commissioning and the survey's own arithmetic \
             wants {} ms; a budget tighter than the run is a red 124 on a tour that would have \
             finished",
            allowance_ns / 1_000_000,
            derived_ns / 1_000_000,
        );
    }

    #[test]
    fn the_first_motion_waits_out_the_engagement_and_the_raise() {
        let tour = Tour::of(&two()).expect("a two-motion library");
        let play_ns = i64::try_from(tour.legs()[0].script.steps()[1].after_ms)
            .expect("a planned offset is a count of ms")
            * 1_000_000;
        let engage_ns = engage_allowance_cycles() * PERIOD_NS;
        let raise_ns = up_clocks().cycles() * PERIOD_NS;
        assert!(
            play_ns >= engage_ns + raise_ns,
            "the first motion opens at {} ms; taking hold of the machine allows {} ms and the \
             raise itself runs {} ms, and a motion composed over a base still rising is a \
             different reading than the one this run takes",
            play_ns / 1_000_000,
            engage_ns / 1_000_000,
            raise_ns / 1_000_000,
        );
    }

    #[test]
    fn every_leg_compiles_against_the_table_it_was_built_from() {
        let table = two();
        let mut edge = Edge::new(EdgeConfig::for_pod(ASK_POD), table.clone());
        let tour = Tour::of(&table).expect("a two-motion library");
        for (index, leg) in tour.legs().iter().enumerate() {
            let accepted = edge
                .accept(
                    leg.script.encode().as_bytes(),
                    clockwork_rs::SyncTime::from_nanos(
                        ARRIVAL_NS + i64::try_from(index).expect("a small fixture") * 1_000_000_000,
                    ),
                )
                .expect("every name the tour plays is one its own edge resolves");
            assert_eq!(accepted.seq, leg.script.seq());
            assert_eq!(
                accepted.message.overlays().len(),
                1,
                "one motion per script, so one window",
            );
        }
    }

    #[test]
    fn a_motion_the_table_does_not_hold_is_not_a_tour_to_begin_with() {
        // The plan is the table, so there is no way to name a motion the edge
        // would refuse; what there is a way to do is hand it no table at all.
        let refused = Tour::of(&MotionTable::default()).expect_err("an empty library");
        assert!(refused.contains("no motions"), "{refused}");
    }

    #[test]
    fn the_budget_is_the_plan_plus_the_allowances() {
        let tour = Tour::of(&two()).expect("a two-motion library");
        let dispatch = u64::try_from(DISPATCH_ALLOWANCE.as_millis()).expect("a count of ms");
        let expected = COMMISSIONING_ALLOWANCE_MS
            + tour.legs()[0].window_close_ms
            + 2 * dispatch
            + tour.legs()[1].stow_end_ms
            + RELEASE_ALLOWANCE_MS
            + END_MARGIN_MS;
        assert_eq!(
            tour.budget().as_secs(),
            expected.div_ceil(1000) + BUDGET_MARGIN_S,
        );
    }

    #[test]
    fn a_longer_library_needs_a_longer_budget() {
        let short = Tour::of(&two()).expect("two motions");
        let long = Tour::of(&table(&[
            ("a", 4000, 200),
            ("b", 2500, 120),
            ("c", 9000, 200),
        ]))
        .expect("three motions");
        assert!(long.budget() > short.budget());
    }

    /// A listener that answers one request with `status` and hands back what it
    /// was sent.
    fn launcher(status: &'static str) -> (SocketAddr, thread::JoinHandle<String>) {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("an ephemeral port");
        let address = listener
            .local_addr()
            .expect("a bound listener has an address");
        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("the one request");
            let mut request = [0u8; 512];
            let read = stream.read(&mut request).expect("the request's bytes");
            stream
                .write_all(status.as_bytes())
                .expect("the launcher answers");
            String::from_utf8_lossy(&request[..read]).into_owned()
        });
        (address, handle)
    }

    #[test]
    fn the_quit_is_one_post_the_launcher_answers() {
        let (address, handle) = launcher("HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n");
        quit_launcher(address, Duration::ZERO, &AtomicBool::new(false))
            .expect("a launcher that took the quit");
        let request = handle.join().expect("the listener thread");
        assert!(request.starts_with("POST /quit HTTP/1.1\r\n"), "{request}");
        assert!(request.contains("Connection: close"), "{request}");
        assert!(request.ends_with("\r\n\r\n"), "{request}");
    }

    #[test]
    fn a_quit_the_launcher_did_not_take_is_a_red_of_its_own() {
        let (address, handle) = launcher("HTTP/1.1 500 Internal Server Error\r\n\r\n");
        let refused = quit_launcher(address, Duration::ZERO, &AtomicBool::new(false))
            .expect_err("a launcher that refused the quit");
        handle.join().expect("the listener thread");
        assert!(refused.contains("did not accept the quit"), "{refused}");
        assert!(refused.contains("500"), "{refused}");
    }

    #[test]
    fn a_launcher_that_is_not_there_is_a_red_naming_the_address() {
        // A port nothing holds: bound and dropped, so the number is known free
        // for as long as this case takes.
        let address = {
            let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("an ephemeral port");
            listener
                .local_addr()
                .expect("a bound listener has an address")
        };
        let refused = quit_launcher(address, Duration::ZERO, &AtomicBool::new(false))
            .expect_err("nothing is listening");
        assert!(refused.contains(&address.to_string()), "{refused}");
    }

    #[test]
    fn a_quit_that_beat_the_launcher_up_waits_for_it() {
        // The port is bound, its number read off, and let go: the listener
        // stands up on it a moment later, which is the shell's own order --
        // sender first, launcher after.
        let address = {
            let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("an ephemeral port");
            listener
                .local_addr()
                .expect("a bound listener has an address")
        };
        let handle = thread::spawn(move || {
            thread::sleep(Duration::from_millis(200));
            let listener = TcpListener::bind(address).expect("the launcher binds its control port");
            let (mut stream, _) = listener.accept().expect("the one request");
            let mut request = [0u8; 512];
            let read = stream.read(&mut request).expect("the request's bytes");
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
                .expect("the launcher answers");
            String::from_utf8_lossy(&request[..read]).into_owned()
        });
        quit_launcher(address, Duration::from_secs(5), &AtomicBool::new(false))
            .expect("a launcher that came up while the quit was offering itself");
        let request = handle.join().expect("the listener thread");
        assert!(
            request.starts_with("POST /quit"),
            "an ending reached before the launcher bound its port must still stop the run, or \
             the whole backstop is spent on a machine nobody asked to move: {request}",
        );
    }

    #[test]
    fn a_stop_while_the_quit_waits_for_the_launcher_ends_the_wait() {
        // A port nothing holds, and a stop already set: the shell only sends
        // one once the launcher has returned, so waiting out the connect window
        // would be waiting for a process that has exited while the shell waits
        // on this one.
        let address = {
            let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("an ephemeral port");
            listener
                .local_addr()
                .expect("a bound listener has an address")
        };
        let stopped = AtomicBool::new(true);
        let began = std::time::Instant::now();
        let refused = quit_launcher(address, Duration::from_secs(15), &stopped)
            .expect_err("a launcher that has already gone");
        assert!(refused.contains("had already gone"), "{refused}");
        assert!(
            began.elapsed() < Duration::from_secs(5),
            "the stop cuts the connect window short rather than waiting it out",
        );
    }

    #[test]
    fn the_control_address_is_the_launchers_default() {
        let address: SocketAddr = LAUNCHER_CONTROL
            .parse()
            .expect("the constant is an address, and the sender parses it once at start-up");
        assert_eq!(address.port(), 8080);
        assert!(address.ip().is_loopback(), "the API is never off the unit");
    }
}
