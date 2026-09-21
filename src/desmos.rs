//! Formatting numbers as Desmos list literals.
//!
//! Shared by the collectors that dump data for offline fitting or plotting:
//! [`sysid`](crate::sysid) and [`motion_profile`](crate::motion_profile). Desmos
//! is picky about what it will accept pasted into an expression, so all of that
//! pickiness lives here:
//!
//! - Numbers are fixed-decimal. Rust's default `{}` switches to scientific
//!   notation for small magnitudes (`1.2e-3`), which Desmos cannot read inside a
//!   list.
//! - Entries are comma-separated with no spaces, and the whole literal is one
//!   line, so a list pastes as a single expression.
//!
//! A formatter returns the `[..]` literal only; the caller supplies the name it
//! is being assigned to (`x_1=`, `L=`, ..).

use std::fmt::Write;

/// Decimal places every emitted number carries.
const PLACES: usize = 4;

/// A list of scalars: `[1.0000,2.5000]`.
pub fn list(values: impl IntoIterator<Item = f64>) -> String {
    let mut out = String::from("[");
    for (index, value) in values.into_iter().enumerate() {
        if index > 0 {
            out.push(',');
        }
        let _ = write!(out, "{value:.PLACES$}");
    }
    out.push(']');
    out
}

/// A list of points: `[(0.0000,1.0000),(0.0100,1.5000)]`.
pub fn points(points: impl IntoIterator<Item = (f64, f64)>) -> String {
    let mut out = String::from("[");
    for (index, (x, y)) in points.into_iter().enumerate() {
        if index > 0 {
            out.push(',');
        }
        let _ = write!(out, "({x:.PLACES$},{y:.PLACES$})");
    }
    out.push(']');
    out
}

#[cfg(test)]
mod tests {
    use super::{list, points};

    #[test]
    fn empty_lists_are_still_valid_literals() {
        assert_eq!(list([]), "[]");
        assert_eq!(points([]), "[]");
    }

    #[test]
    fn scalars_are_comma_separated_and_fixed_decimal() {
        assert_eq!(list([0.0, -1.5, 2.25]), "[0.0000,-1.5000,2.2500]");
    }

    #[test]
    fn small_magnitudes_avoid_scientific_notation() {
        // `{}` would render this as `1.2e-7`, which Desmos rejects in a list.
        assert_eq!(list([0.00000012]), "[0.0000]");
    }

    #[test]
    fn points_are_parenthesized_pairs() {
        assert_eq!(points([(0.0, 1.0), (0.01, -2.0)]), "[(0.0000,1.0000),(0.0100,-2.0000)]");
    }
}
