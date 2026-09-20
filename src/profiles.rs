//! Motion profiles generated offline by vmplib.
//!
//! One module per profile, each holding nothing but a `SAMPLES` constant that
//! [`motion_profile::follow`](crate::motion_profile::follow) replays. Generated
//! code lives here so that regenerating a path is a file drop rather than an
//! edit.
//!
//! # Adding a profile
//!
//! Drop vmplib's output in as `src/profiles/<name>.rs` and add a `pub mod
//! <name>;` line below. vmplib emits its own copy of the `DriveSample`
//! definition at the top of the file; delete those lines and import the shared
//! one instead, so that every profile is the same type and the follower accepts
//! all of them:
//!
//! ```ignore
//! use crate::motion_profile::DriveSample;
//! ```
//!
//! Nothing else about the generated file needs to change. The field names,
//! units, and ordering vmplib emits already match [`DriveSample`].
//!
//! [`DriveSample`]: crate::motion_profile::DriveSample

pub mod example;
