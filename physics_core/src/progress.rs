// Tier: F
//! Run progress of a campaign: how much simulated time each test has done, and in which state it is. A read-only side channel for the
//! helper programs' `UAVPROGRESS` lines; nothing here is read by the physics, so the results are bit-identical with or without it.
//!
//! The counters are plain relaxed atomics, one cache line per test, indexed by the test's POSITION in the campaign's test list
//! (at most `MAX_TESTS`). A worker thread names the test it is running with `set_current`; every `Sim` built on that thread
//! remembers the index and adds one step (`DT_S` of simulated time) per `Sim::step`.
use std::cell::Cell;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};
use std::time::Instant;

// These are `const` items of the private kind on purpose: cbindgen turns every `pub const` of the crate into a C `#define`, and names
// like DONE, NONE or RUNNING would then collide with Unreal's macros in drone_ffi.h. The public names are the functions below.
const MAX_TESTS: usize = 8;
const NONE: usize = usize::MAX;
const WAITING: u8 = 0;
const RUNNING: u8 = 1;
const DONE: u8 = 2;
const FAILED: u8 = 3;

/// Most tests a campaign has (the test library has eight).
#[must_use]
pub const fn max_tests() -> usize {
    MAX_TESTS
}
/// "No test": the index of a thread that is not running a campaign test.
#[must_use]
pub const fn no_test() -> usize {
    NONE
}
/// State: not started.
#[must_use]
pub const fn state_waiting() -> u8 {
    WAITING
}
/// State: running.
#[must_use]
pub const fn state_running() -> u8 {
    RUNNING
}
/// State: finished with a result.
#[must_use]
pub const fn state_done() -> u8 {
    DONE
}
/// State: stopped by an error.
#[must_use]
pub const fn state_failed() -> u8 {
    FAILED
}

#[repr(align(64))]
struct Slot {
    steps: AtomicU64,
    state: AtomicU8,
    start_us: AtomicU64,
    end_us: AtomicU64,
}

#[allow(clippy::declare_interior_mutable_const)]
const SLOT: Slot = Slot {
    steps: AtomicU64::new(0),
    state: AtomicU8::new(WAITING),
    start_us: AtomicU64::new(0),
    end_us: AtomicU64::new(0),
};

/// The counters of one campaign. The process has one (`GLOBAL`, used by the free functions below); tests make their own.
pub struct Tracker {
    slots: [Slot; MAX_TESTS],
    epoch: OnceLock<Instant>,
}

impl Tracker {
    /// An empty tracker.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slots: [SLOT; MAX_TESTS],
            epoch: OnceLock::new(),
        }
    }

    fn now_us(&self) -> u64 {
        let epoch = self.epoch.get_or_init(Instant::now);
        u64::try_from(epoch.elapsed().as_micros()).unwrap_or(u64::MAX)
    }

    /// Clears all counters.
    pub fn reset(&self) {
        let _ = self.epoch.get_or_init(Instant::now);
        for s in &self.slots {
            s.steps.store(0, Ordering::Relaxed);
            s.state.store(WAITING, Ordering::Relaxed);
            s.start_us.store(0, Ordering::Relaxed);
            s.end_us.store(0, Ordering::Relaxed);
        }
    }

    /// Adds one simulated step to a test (out-of-range and `NONE` indices are ignored).
    pub fn add_step(&self, index: usize) {
        if let Some(s) = self.slots.get(index) {
            s.steps.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Sets a test's state and records the start / end time.
    pub fn set_state(&self, index: usize, state: u8) {
        if let Some(s) = self.slots.get(index) {
            match state {
                RUNNING => s.start_us.store(self.now_us(), Ordering::Relaxed),
                DONE | FAILED => s.end_us.store(self.now_us(), Ordering::Relaxed),
                _ => {}
            }
            s.state.store(state, Ordering::Relaxed);
        }
    }

    /// Steps done so far by a test (0 for an out-of-range index).
    #[must_use]
    pub fn steps(&self, index: usize) -> u64 {
        self.slots
            .get(index)
            .map_or(0, |s| s.steps.load(Ordering::Relaxed))
    }

    /// State of a test (`WAITING` for an out-of-range index).
    #[must_use]
    pub fn state(&self, index: usize) -> u8 {
        self.slots
            .get(index)
            .map_or(WAITING, |s| s.state.load(Ordering::Relaxed))
    }

    /// Wall-clock start and end of a test, microseconds since the first use (0 when not reached).
    #[must_use]
    pub fn wall_us(&self, index: usize) -> (u64, u64) {
        self.slots.get(index).map_or((0, 0), |s| {
            (
                s.start_us.load(Ordering::Relaxed),
                s.end_us.load(Ordering::Relaxed),
            )
        })
    }
}

impl Default for Tracker {
    fn default() -> Self {
        Self::new()
    }
}

static GLOBAL: Tracker = Tracker::new();

thread_local! {
    static CURRENT: Cell<usize> = const { Cell::new(NONE) };
}

/// Clears the process-wide counters (call before a campaign).
pub fn reset() {
    GLOBAL.reset();
}

/// Names the test the current thread is running (`NONE` to clear).
pub fn set_current(index: usize) {
    CURRENT.with(|c| c.set(index));
}

/// Index of the test the current thread is running, or `NONE`.
#[must_use]
pub fn current() -> usize {
    CURRENT.with(Cell::get)
}

/// Adds one simulated step to a test of the process-wide tracker.
pub fn add_step(index: usize) {
    GLOBAL.add_step(index);
}

/// Sets a test's state in the process-wide tracker.
pub fn set_state(index: usize, state: u8) {
    GLOBAL.set_state(index, state);
}

/// Steps done so far by a test (process-wide tracker).
#[must_use]
pub fn steps(index: usize) -> u64 {
    GLOBAL.steps(index)
}

/// State of a test (process-wide tracker).
#[must_use]
pub fn state(index: usize) -> u8 {
    GLOBAL.state(index)
}

/// Wall-clock start and end of a test, microseconds since the first use.
#[must_use]
pub fn wall_us(index: usize) -> (u64, u64) {
    GLOBAL.wall_us(index)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_progress_counts_per_index_and_ignores_bad_indices() {
        // Hand check on a LOCAL tracker (campaign tests running in parallel use the global one): three steps on index 1, one on
        // index 2; NONE and an index past the end change nothing.
        let t = Tracker::new();
        t.reset();
        for _ in 0..3 {
            t.add_step(1);
        }
        t.add_step(2);
        t.add_step(NONE);
        t.add_step(MAX_TESTS);
        assert_eq!((t.steps(0), t.steps(1), t.steps(2)), (0, 3, 1));
        assert_eq!(t.steps(MAX_TESTS), 0);
        t.set_state(1, RUNNING);
        t.set_state(1, DONE);
        assert_eq!(t.state(1), DONE);
        assert_eq!(t.state(0), WAITING);
        let (a, b) = t.wall_us(1);
        assert!(b >= a);
        set_current(4);
        assert_eq!(current(), 4);
        set_current(NONE);
        assert_eq!(current(), NONE);
    }
}
