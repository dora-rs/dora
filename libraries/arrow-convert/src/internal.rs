//! Raw access to the Arrow array inside a [`DoraArray`], for dora's own crates.
//!
//! # Not public API
//!
//! Everything in this module names dora's **internal** Arrow major and is
//! **exempt from dora's semver guarantee**. It changes — silently, in a minor
//! release — whenever dora bumps its internal Arrow. It is `pub` only because
//! `DoraArray` lives in this crate while the code that has to build and unwrap
//! it (`dora-node-api`, the C/C++/Python bindings, the record/replay nodes)
//! lives in others; Rust has no cross-crate `pub(crate)`.
//!
//! This module is deliberately **not** re-exported from `dora-node-api`, so it
//! is not reachable from the frozen 1.x surface. Do not depend on it from
//! outside the dora workspace. If you need the Arrow array, enable the feature
//! that names your major (`arrow-v59`, `arrow-v58`, …) and use
//! [`DoraArray::as_array`](crate::DoraArray::as_array) /
//! [`DoraArray::to_arrow_v58`](crate::DoraArray::to_arrow_v58), which are
//! version-honest and therefore cannot drift under you.

use crate::DoraArray;

/// Wrap an Arrow array (internal major) as a dora payload.
pub fn from_array_ref(array: arrow::array::ArrayRef) -> DoraArray {
    DoraArray(array)
}

/// Wrap Arrow [`ArrayData`](arrow::array::ArrayData) as a dora payload.
pub fn from_array_data(data: arrow::array::ArrayData) -> DoraArray {
    DoraArray(arrow::array::make_array(data))
}

/// Borrow the Arrow array (internal major) inside a dora payload.
pub fn array_ref(data: &DoraArray) -> &arrow::array::ArrayRef {
    &data.0
}

/// Take the Arrow array (internal major) out of a dora payload.
pub fn into_array_ref(data: DoraArray) -> arrow::array::ArrayRef {
    data.0
}
