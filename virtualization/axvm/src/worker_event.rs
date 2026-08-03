//! Lock-free event sequence for deferred host workers.

use core::sync::atomic::{AtomicU64, Ordering};

/// Monotonic event sequence shared by producers and one deferred worker.
///
/// Unlike a ready bit that the worker clears, a sequence cannot lose an event
/// that races with the worker taking its snapshot. Producers advance the
/// sequence before waking the worker; the worker snapshots it before draining
/// work and waits until a later sequence is observed.
#[derive(Default)]
pub struct WorkerEventSequence {
    sequence: AtomicU64,
}

impl WorkerEventSequence {
    /// Creates a sequence with no published events.
    pub const fn new() -> Self {
        Self {
            sequence: AtomicU64::new(0),
        }
    }

    /// Publishes an event before the producer wakes the worker.
    pub fn advance(&self) {
        self.sequence.fetch_add(1, Ordering::Release);
    }

    /// Returns the current sequence for the worker's next snapshot.
    pub fn current(&self) -> u64 {
        self.sequence.load(Ordering::Acquire)
    }

    /// Returns whether an event was published after `observed`.
    pub fn has_advanced_since(&self, observed: u64) -> bool {
        self.current() != observed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_arriving_after_worker_snapshot_remains_observable() {
        let events = WorkerEventSequence::new();
        let observed = events.current();

        // This is the old ready-bit clear window: the producer publishes after
        // the worker takes its snapshot but before the next wait begins.
        events.advance();

        assert!(events.has_advanced_since(observed));
    }

    #[test]
    fn snapshot_consumes_only_events_already_observed() {
        let events = WorkerEventSequence::new();
        events.advance();
        let observed = events.current();

        assert!(!events.has_advanced_since(observed));
        events.advance();
        assert!(events.has_advanced_since(observed));
    }
}
