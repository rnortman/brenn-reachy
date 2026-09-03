//! The modelled servos' response delay: what a lagged row is chasing.
//!
//! The plant's ordinary row closes a fixed fraction of the gap to its target
//! every cycle, so it never lags a goal it is not jammed against. A real servo
//! does: its loop takes time to turn round, and the distance it stands behind a
//! moving goal is the whole reason the motion tick's tracking window exists. A
//! lagged row therefore chases the target it was given a fixed number of cycles
//! ago, and what that costs is nine rings of past targets.
//!
//! One ring per bus row, all of one depth, flat and row-major for the reason the
//! register file is: a slot carries an array, not an array of arrays. The depth
//! is the longest delay a scenario may ask for, and a scenario asking for more
//! is refused rather than quietly given less -- a run whose premise is a servo
//! 19 cycles behind says nothing if the plant gave it 8.
//!
//! Nothing here decides anything about the machine. Which rows are lagged is
//! the scenario's injection and how far a row then moves is the plant's slew;
//! this module is the storage and the arithmetic of walking it.

use reachy_motion::joints::ROW_COUNT;

/// How many cycles of past targets a ring holds.
///
/// The longest response delay a scenario can state, and the reason it is this
/// number: the antenna lag the recordings carry is about two thirds of a second
/// of travel at the rate a raise commands, which is a little over thirty cycles
/// of this grid. A power of two so the wrap is exact at every depth a scenario
/// might read out of it.
pub const LAG_DEPTH: usize = 32;

/// How many cells the nine rings hold together.
pub const HISTORY_CELLS: usize = ROW_COUNT * LAG_DEPTH;

/// Whether a delay of `cycles` fits the rings.
///
/// A delay of zero is a row that follows at once, which is what an unlagged row
/// already does; it fits, and asking for it is how a scenario ends a lag it set
/// without renaming the rows.
#[must_use]
pub fn representable(cycles: u32) -> bool {
    (cycles as usize) < LAG_DEPTH
}

/// Fill `row`'s whole ring with `angle`.
///
/// Run when a row is given a response delay it did not already have, at the
/// target it holds then -- a row already lagged keeps the history it has. Every
/// cell of a lagged row's ring is therefore a target this row was actually
/// given, so a delay reaching back further than the delay has existed chases
/// that target rather than the zero an unwritten slot holds -- which on this
/// machine is a real angle, and a plant that slewed to it would be moving on
/// nobody's command.
pub fn seed(cells: &mut [f64; HISTORY_CELLS], row: usize, angle: f64) {
    let Some(ring) = ring_mut(cells, row) else {
        return;
    };
    ring.fill(angle);
}

/// Walk every ring on one cycle, writing `targets` into the new cells.
///
/// The cursor is answered rather than stored: the caller owns the slot's field,
/// and a cursor written by two places is a ring two things disagree about the
/// age of.
#[must_use]
pub fn push(cells: &mut [f64; HISTORY_CELLS], cursor: u32, targets: &[f64; ROW_COUNT]) -> u32 {
    let next = (cursor as usize + 1) % LAG_DEPTH;
    for (row, target) in targets.iter().enumerate() {
        if let Some(ring) = ring_mut(cells, row) {
            ring[next] = *target;
        }
    }
    next as u32
}

/// What `row` was being asked for `cycles` cycles ago, if the rings hold it.
///
/// `None` for a row this build does not place or a delay past the ring's depth:
/// the caller's answer to either is the target the row holds now, which is the
/// unlagged plant.
#[must_use]
pub fn delayed(cells: &[f64; HISTORY_CELLS], cursor: u32, row: usize, cycles: u32) -> Option<f64> {
    if !representable(cycles) {
        return None;
    }
    let ring = ring(cells, row)?;
    let cell = (cursor as usize + LAG_DEPTH - cycles as usize) % LAG_DEPTH;
    ring.get(cell).copied()
}

/// One row's ring.
fn ring(cells: &[f64; HISTORY_CELLS], row: usize) -> Option<&[f64]> {
    cells.get(row * LAG_DEPTH..(row + 1) * LAG_DEPTH)
}

/// The same, to write.
fn ring_mut(cells: &mut [f64; HISTORY_CELLS], row: usize) -> Option<&mut [f64]> {
    cells.get_mut(row * LAG_DEPTH..(row + 1) * LAG_DEPTH)
}

#[cfg(test)]
mod tests {
    use super::{HISTORY_CELLS, LAG_DEPTH, delayed, push, representable, seed};
    use reachy_motion::joints::ROW_COUNT;

    /// A ring per row, and the rings do not reach into each other: what one row
    /// was asked for says nothing about what another was.
    #[test]
    fn each_row_keeps_its_own_ring() {
        let mut cells = [0.0; HISTORY_CELLS];
        for row in 0..ROW_COUNT {
            seed(&mut cells, row, row as f64);
        }
        let mut targets = [0.0; ROW_COUNT];
        for (row, target) in targets.iter_mut().enumerate() {
            *target = 10.0 + row as f64;
        }
        let cursor = push(&mut cells, 0, &targets);

        for row in 0..ROW_COUNT {
            assert_eq!(delayed(&cells, cursor, row, 0), Some(10.0 + row as f64));
            assert_eq!(delayed(&cells, cursor, row, 1), Some(row as f64));
        }
    }

    /// The cursor wraps and the ring keeps the last `LAG_DEPTH` cycles: a
    /// target older than the ring is one the arithmetic must not answer with a
    /// newer cell.
    #[test]
    fn the_ring_holds_the_last_depth_cycles_and_wraps() {
        let mut cells = [0.0; HISTORY_CELLS];
        let mut cursor = 0;
        seed(&mut cells, 0, -1.0);
        let mut targets = [0.0; ROW_COUNT];
        // A whole lap and a half, so the cells the second half wrote are the
        // ones the reads below have to find.
        for step in 0..(LAG_DEPTH + LAG_DEPTH / 2) {
            targets[0] = step as f64;
            cursor = push(&mut cells, cursor, &targets);
        }
        let newest = (LAG_DEPTH + LAG_DEPTH / 2 - 1) as f64;
        assert_eq!(delayed(&cells, cursor, 0, 0), Some(newest));
        assert_eq!(
            delayed(&cells, cursor, 0, (LAG_DEPTH - 1) as u32),
            Some(newest - (LAG_DEPTH - 1) as f64),
        );
        assert!(!representable(LAG_DEPTH as u32));
        assert_eq!(delayed(&cells, cursor, 0, LAG_DEPTH as u32), None);
    }

    /// A row nothing has commanded reads its seed at every depth: a delay never
    /// chases a cell nobody wrote.
    #[test]
    fn a_seeded_ring_reads_the_same_target_at_every_depth() {
        let mut cells = [0.0; HISTORY_CELLS];
        seed(&mut cells, 3, 1.25);
        for cycles in 0..LAG_DEPTH as u32 {
            assert_eq!(delayed(&cells, 0, 3, cycles), Some(1.25));
        }
    }

    /// A row this build does not place has no ring, and the caller is told so
    /// rather than handed another row's cell.
    #[test]
    fn a_row_past_the_last_one_has_no_ring() {
        let cells = [0.0; HISTORY_CELLS];
        assert_eq!(delayed(&cells, 0, ROW_COUNT, 0), None);
    }
}
