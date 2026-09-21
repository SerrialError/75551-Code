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
