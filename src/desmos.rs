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
//!
//! Dumps are printed with [`print_paced`], not `print!`: they run to tens of
//! kilobytes, and the wireless terminal path loses data when it's written at
//! full speed.

use std::{
    fmt::Write as _,
    io::Write as _,
    time::Duration,
};

use vexide::prelude::sleep;

/// Decimal places every emitted number carries.
const PLACES: usize = 4;

/// Bytes written per [`print_paced`] chunk.
const PACE_CHUNK_BYTES: usize = 64;

/// Pause after each [`print_paced`] chunk: 64 B / 50 ms ≈ 1.3 KB/s.
//
// Why pace at all: over a controller (VEXnet or Bluetooth), `cargo v5 terminal`
// can't stream stdout. It polls the brain with one `UserDataPacket` round trip
// per read (see `vex-v5-serial`'s `read_user`), sleeping 10 ms between polls,
// with a 100 ms timeout per round trip. Brain-side, std's `Stdout` only blocks
// on the 2 KB VEXos serial FIFO, and that FIFO doesn't wait for the radio, so a
// single `print!` of a whole dump overran the link. A 55 KB sysid dump came
// back with whole blocks missing and numbers spliced mid-digit.
//
// VEX doesn't publish a sustained throughput for this path, so the rate is
// deliberately conservative: a full sysid dump takes about 45 s. If a dump
// still comes back corrupted, raise the interval. Over a USB cable to the
// brain, this rate is far below what the link can take, so pacing is harmless
// there, just slower than it needs to be.
const PACE_INTERVAL: Duration = Duration::from_millis(50);

/// Prints `text` to stdout slowly enough for the wireless terminal to keep up:
/// [`PACE_CHUNK_BYTES`] at a time, each one flushed and followed by an async
/// sleep of [`PACE_INTERVAL`], so other tasks keep running while it prints.
///
/// Brackets the text with a start line (size and time estimate) and an end
/// marker. If the end marker never shows up, the dump was cut off.
/// `text` is expected to be ASCII (it's numbers and labels). A chunk boundary
/// could otherwise split a UTF-8 character across terminal packets.
pub async fn print_paced(text: &str) {
    let chunks = text.len().div_ceil(PACE_CHUNK_BYTES);
    let seconds = chunks as f64 * PACE_INTERVAL.as_secs_f64();
    println!(
        "\n# printing {} bytes, ~{seconds:.0} s - wait for '# end of dump'",
        text.len()
    );

    for chunk in text.as_bytes().chunks(PACE_CHUNK_BYTES) {
        let mut stdout = std::io::stdout();
        // std's stdout is line-buffered, so a chunk without a newline would sit
        // in its buffer. Flush to hand every chunk to VEXos right away.
        let _ = stdout.write_all(chunk);
        let _ = stdout.flush();
        sleep(PACE_INTERVAL).await;
    }

    println!("\n# end of dump");
}

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

/// A list of integers, without decimal places: `[1,1,2]`. For indices, where
/// the trailing `.0000` would only bloat the dump.
pub fn int_list(values: impl IntoIterator<Item = usize>) -> String {
    let mut out = String::from("[");
    for (index, value) in values.into_iter().enumerate() {
        if index > 0 {
            out.push(',');
        }
        let _ = write!(out, "{value}");
    }
    out.push(']');
    out
}

/// A list of points:`[(0.0000,1.0000),(0.0100,1.5000)]`.
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
