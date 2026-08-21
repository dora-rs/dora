/// Tracks a run of consecutive send timeouts to a bounded-channel WS
/// subscriber.
///
/// A transiently-slow `dora logs -f` / topic-debug reader can keep its channel
/// full for a single 100 ms send window; that must not evict it. Both
/// `LogSubscriber` and `TopicSubscriber` embed this counter and only close a
/// subscriber once the streak reaches their eviction threshold, so the two
/// paths share one definition of "how many timeouts in a row is too many".
#[derive(Debug, Default)]
pub(crate) struct TimeoutStreak {
    consecutive: usize,
}

impl TimeoutStreak {
    /// Record a timeout and return the new consecutive-timeout count.
    pub(crate) fn record(&mut self) -> usize {
        self.consecutive += 1;
        self.consecutive
    }

    /// Reset the streak after a successful send.
    pub(crate) fn reset(&mut self) {
        self.consecutive = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::TimeoutStreak;

    #[test]
    fn record_is_monotonic_and_reset_returns_to_zero() {
        let mut streak = TimeoutStreak::default();
        assert_eq!(streak.record(), 1);
        assert_eq!(streak.record(), 2);
        assert_eq!(streak.record(), 3);
        streak.reset();
        assert_eq!(streak.record(), 1);
    }
}
