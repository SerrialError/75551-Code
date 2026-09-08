//! Small, self-contained scalar filters used by the motor
//! [`VelocityEstimator`](crate::velocity_estimator).
//!
//! Each filter operates on a stream of `f64` samples fed one at a time through
//! `filter`. The windowed filters ([`Sma`], [`Median`], [`MaxAbs`]) start empty
//! and, until their window fills, operate over however many samples they've seen
//! so far rather than waiting for a full window.
//!
//! This module is deliberately free of any `vexide`/hardware dependency so it can
//! be unit-tested on the host.

use std::collections::VecDeque;

/// A simple moving average: the mean of the last `window` samples.
pub struct Sma {
    window: usize,
    samples: VecDeque<f64>,
}

impl Sma {
    pub fn new(window: usize) -> Self {
        Self {
            window: window.max(1),
            samples: VecDeque::with_capacity(window.max(1)),
        }
    }

    /// Pushes `input` and returns the mean of the samples currently in the
    /// window.
    pub fn filter(&mut self, input: f64) -> f64 {
        if self.samples.len() == self.window {
            self.samples.pop_front();
        }
        self.samples.push_back(input);
        self.samples.iter().sum::<f64>() / self.samples.len() as f64
    }
}

/// A median filter: the middle value of the last `window` samples, sorted.
pub struct Median {
    window: usize,
    samples: VecDeque<f64>,
}

impl Median {
    pub fn new(window: usize) -> Self {
        Self {
            window: window.max(1),
            samples: VecDeque::with_capacity(window.max(1)),
        }
    }

    /// Pushes `input` and returns the middle of the sorted window (the lower of
    /// the two middles while the window holds an even number of samples).
    pub fn filter(&mut self, input: f64) -> f64 {
        if self.samples.len() == self.window {
            self.samples.pop_front();
        }
        self.samples.push_back(input);

        let mut sorted: Vec<f64> = self.samples.iter().copied().collect();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        sorted[sorted.len() / 2]
    }
}

/// Tracks the largest absolute value seen in the last `window` samples.
pub struct MaxAbs {
    window: usize,
    samples: VecDeque<f64>,
}

impl MaxAbs {
    pub fn new(window: usize) -> Self {
        Self {
            window: window.max(1),
            samples: VecDeque::with_capacity(window.max(1)),
        }
    }

    /// Pushes `input` and returns the largest `abs()` currently in the window.
    pub fn filter(&mut self, input: f64) -> f64 {
        if self.samples.len() == self.window {
            self.samples.pop_front();
        }
        self.samples.push_back(input);
        self.samples
            .iter()
            .map(|value| value.abs())
            .fold(0.0, f64::max)
    }
}

/// An exponential moving average: `state = input * gain + state * (1 - gain)`.
///
/// The `gain` is supplied per sample rather than stored, so a caller can vary it
/// (as the estimator does with its acceleration-adaptive gain).
#[derive(Default)]
pub struct Ema {
    state: f64,
}

impl Ema {
    pub fn new() -> Self {
        Self { state: 0.0 }
    }

    pub fn filter(&mut self, input: f64, gain: f64) -> f64 {
        self.state = input * gain + self.state * (1.0 - gain);
        self.state
    }
}

/// A discrete derivative: `(input - previous_input) / dt_ms`.
///
/// Returns the previous result when `dt_ms <= 0` (no time has passed, so the
/// rate is undefined) and `0.0` for the very first sample (no previous input to
/// difference against).
#[derive(Default)]
pub struct Derivative {
    previous_input: Option<f64>,
    previous_result: f64,
}

impl Derivative {
    pub fn new() -> Self {
        Self {
            previous_input: None,
            previous_result: 0.0,
        }
    }

    pub fn filter(&mut self, input: f64, dt_ms: f64) -> f64 {
        if dt_ms <= 0.0 {
            return self.previous_result;
        }
        let result = match self.previous_input {
            Some(previous) => (input - previous) / dt_ms,
            None => 0.0,
        };
        self.previous_input = Some(input);
        self.previous_result = result;
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    #[test]
    fn sma_averages_partial_then_full_window() {
        let mut sma = Sma::new(3);
        // Window fills gradually, averaging over what it holds so far.
        assert!((sma.filter(3.0) - 3.0).abs() < EPS);
        assert!((sma.filter(5.0) - 4.0).abs() < EPS);
        assert!((sma.filter(7.0) - 5.0).abs() < EPS);
        // Oldest (3.0) drops out: mean of 5,7,9.
        assert!((sma.filter(9.0) - 7.0).abs() < EPS);
    }

    #[test]
    fn median_returns_middle_of_sorted_window() {
        let mut median = Median::new(7);
        let mut last = 0.0;
        // Values 0,2,4,6,8,10,12 in scrambled order; sorted middle is 6.
        for value in [10.0, 2.0, 8.0, 4.0, 6.0, 0.0, 12.0] {
            last = median.filter(value);
        }
        assert!((last - 6.0).abs() < EPS);
    }

    #[test]
    fn median_uses_partial_window_before_it_fills() {
        let mut median = Median::new(7);
        // Two samples held: sorted [3, 9], middle index 1 -> 9.
        median.filter(9.0);
        assert!((median.filter(3.0) - 9.0).abs() < EPS);
    }

    #[test]
    fn median_rejects_a_single_spike() {
        let mut median = Median::new(7);
        let mut last = 0.0;
        for value in [1.0, 1.0, 1.0, 1000.0, 1.0, 1.0, 1.0] {
            last = median.filter(value);
        }
        // The lone 1000.0 spike is discarded by the median.
        assert!((last - 1.0).abs() < EPS);
    }

    #[test]
    fn max_abs_tracks_largest_magnitude_in_window() {
        let mut max_abs = MaxAbs::new(3);
        assert!((max_abs.filter(1.0) - 1.0).abs() < EPS); // [1]
        assert!((max_abs.filter(-5.0) - 5.0).abs() < EPS); // [1,-5]
        assert!((max_abs.filter(2.0) - 5.0).abs() < EPS); // [1,-5,2]
        assert!((max_abs.filter(3.0) - 5.0).abs() < EPS); // [-5,2,3]
        // The -5.0 finally falls out of the 3-wide window here.
        assert!((max_abs.filter(4.0) - 4.0).abs() < EPS); // [2,3,4]
    }

    #[test]
    fn ema_blends_input_and_state_by_gain() {
        let mut ema = Ema::new();
        // gain 1.0 -> follows input exactly.
        assert!((ema.filter(10.0, 1.0) - 10.0).abs() < EPS);
        // gain 0.5 -> halfway between input and prior state (10.0).
        assert!((ema.filter(20.0, 0.5) - 15.0).abs() < EPS);
        // gain 0.0 -> holds prior state.
        assert!((ema.filter(999.0, 0.0) - 15.0).abs() < EPS);
    }

    #[test]
    fn derivative_differences_over_dt() {
        let mut derivative = Derivative::new();
        // First sample has no predecessor.
        assert!((derivative.filter(4.0, 2.0) - 0.0).abs() < EPS);
        // (10 - 4) / 2 = 3.
        assert!((derivative.filter(10.0, 2.0) - 3.0).abs() < EPS);
    }

    #[test]
    fn derivative_holds_last_result_when_dt_non_positive() {
        let mut derivative = Derivative::new();
        derivative.filter(0.0, 1.0);
        let last = derivative.filter(6.0, 1.0); // 6.0
        assert!((last - 6.0).abs() < EPS);
        // dt <= 0 returns the previous result and ignores the new input.
        assert!((derivative.filter(1000.0, 0.0) - 6.0).abs() < EPS);
        assert!((derivative.filter(1000.0, -5.0) - 6.0).abs() < EPS);
    }
}
