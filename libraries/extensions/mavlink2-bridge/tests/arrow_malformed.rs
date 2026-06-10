//! Negative tests for `MavlinkArrow::from_record_batch`.
//!
//! `command_long_cmd` and the other writer inputs are public dora inputs
//! — any node in the dataflow can publish to them. The decode path must
//! reject malformed batches with `BridgeError`, not panic, otherwise the
//! whole bridge node aborts on a single bad message.

use arrow::array::{Float32Array, RecordBatch, StringArray, UInt8Array, UInt32Array};
use arrow::datatypes::{DataType, Field, Schema};
use dora_mavlink2_bridge::MavlinkArrow;
use dora_mavlink2_bridge::mavlink::common::COMMAND_LONG_DATA;
use std::sync::Arc;

/// All seven `paramN` fields are f32; we keep a single shared row so each
/// negative variant only has to swap one column out.
fn good_command_long_columns() -> Vec<arrow::array::ArrayRef> {
    vec![
        Arc::new(Float32Array::from(vec![1.0])),
        Arc::new(Float32Array::from(vec![2.0])),
        Arc::new(Float32Array::from(vec![3.0])),
        Arc::new(Float32Array::from(vec![4.0])),
        Arc::new(Float32Array::from(vec![5.0])),
        Arc::new(Float32Array::from(vec![6.0])),
        Arc::new(Float32Array::from(vec![7.0])),
        Arc::new(UInt32Array::from(vec![22u32])), // MAV_CMD_NAV_TAKEOFF
        Arc::new(UInt8Array::from(vec![1u8])),
        Arc::new(UInt8Array::from(vec![1u8])),
        Arc::new(UInt8Array::from(vec![0u8])),
    ]
}

#[test]
fn rejects_wrong_column_type() {
    // Replace `command` (UInt32 per spec) with a String column. The
    // pre-fix code path called `as_primitive::<UInt32Type>()` which
    // panics on type mismatch.
    let schema = Schema::new(vec![
        Field::new("param1", DataType::Float32, false),
        Field::new("param2", DataType::Float32, false),
        Field::new("param3", DataType::Float32, false),
        Field::new("param4", DataType::Float32, false),
        Field::new("param5", DataType::Float32, false),
        Field::new("param6", DataType::Float32, false),
        Field::new("param7", DataType::Float32, false),
        Field::new("command", DataType::Utf8, false),
        Field::new("target_system", DataType::UInt8, false),
        Field::new("target_component", DataType::UInt8, false),
        Field::new("confirmation", DataType::UInt8, false),
    ]);
    let mut cols = good_command_long_columns();
    cols[7] = Arc::new(StringArray::from(vec!["takeoff"]));
    let batch = RecordBatch::try_new(Arc::new(schema), cols).expect("build batch");

    let err = COMMAND_LONG_DATA::from_record_batch(&batch).expect_err("expected decode error");
    let msg = err.to_string();
    assert!(
        msg.contains("command") && msg.contains("wrong type"),
        "unexpected error: {msg}"
    );
}

#[test]
fn rejects_zero_row_batch() {
    // Same schema, but every column has zero rows. `value(0)` would
    // panic out-of-bounds; we want a decode error instead.
    let schema = COMMAND_LONG_DATA::schema();
    let cols: Vec<arrow::array::ArrayRef> = vec![
        Arc::new(Float32Array::from(Vec::<f32>::new())),
        Arc::new(Float32Array::from(Vec::<f32>::new())),
        Arc::new(Float32Array::from(Vec::<f32>::new())),
        Arc::new(Float32Array::from(Vec::<f32>::new())),
        Arc::new(Float32Array::from(Vec::<f32>::new())),
        Arc::new(Float32Array::from(Vec::<f32>::new())),
        Arc::new(Float32Array::from(Vec::<f32>::new())),
        Arc::new(UInt32Array::from(Vec::<u32>::new())),
        Arc::new(UInt8Array::from(Vec::<u8>::new())),
        Arc::new(UInt8Array::from(Vec::<u8>::new())),
        Arc::new(UInt8Array::from(Vec::<u8>::new())),
    ];
    let batch = RecordBatch::try_new(Arc::new(schema), cols).expect("build batch");

    let err = COMMAND_LONG_DATA::from_record_batch(&batch).expect_err("expected decode error");
    let msg = err.to_string();
    assert!(
        msg.contains("has 0 rows") && msg.contains("exactly 1"),
        "unexpected error: {msg}"
    );
}

#[test]
fn rejects_multi_row_batch() {
    // Two commands in one batch: row 0 is TAKEOFF, row 1 is LAND. Reading
    // only row 0 would silently drop the LAND command — safety-relevant
    // data loss — so the whole batch must be rejected.
    let schema = COMMAND_LONG_DATA::schema();
    let cols: Vec<arrow::array::ArrayRef> = vec![
        Arc::new(Float32Array::from(vec![1.0, 101.0])),
        Arc::new(Float32Array::from(vec![2.0, 102.0])),
        Arc::new(Float32Array::from(vec![3.0, 103.0])),
        Arc::new(Float32Array::from(vec![4.0, 104.0])),
        Arc::new(Float32Array::from(vec![5.0, 105.0])),
        Arc::new(Float32Array::from(vec![6.0, 106.0])),
        Arc::new(Float32Array::from(vec![7.0, 107.0])),
        // MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_LAND
        Arc::new(UInt32Array::from(vec![22u32, 21u32])),
        Arc::new(UInt8Array::from(vec![1u8, 1u8])),
        Arc::new(UInt8Array::from(vec![1u8, 1u8])),
        Arc::new(UInt8Array::from(vec![0u8, 0u8])),
    ];
    let batch = RecordBatch::try_new(Arc::new(schema), cols).expect("build batch");

    let err = COMMAND_LONG_DATA::from_record_batch(&batch).expect_err("expected decode error");
    let msg = err.to_string();
    assert!(
        msg.contains("has 2 rows") && msg.contains("exactly 1"),
        "unexpected error: {msg}"
    );
}

#[test]
fn rejects_null_row_zero() {
    // First field marked nullable so we can construct a null-at-row-0
    // value without lying about the data type. Pre-fix `value(0)` would
    // happily return whatever zero-bit pattern the slot held.
    let schema = Schema::new(vec![
        Field::new("param1", DataType::Float32, true),
        Field::new("param2", DataType::Float32, false),
        Field::new("param3", DataType::Float32, false),
        Field::new("param4", DataType::Float32, false),
        Field::new("param5", DataType::Float32, false),
        Field::new("param6", DataType::Float32, false),
        Field::new("param7", DataType::Float32, false),
        Field::new("command", DataType::UInt32, false),
        Field::new("target_system", DataType::UInt8, false),
        Field::new("target_component", DataType::UInt8, false),
        Field::new("confirmation", DataType::UInt8, false),
    ]);
    let mut cols = good_command_long_columns();
    cols[0] = Arc::new(Float32Array::from(vec![None]));
    let batch = RecordBatch::try_new(Arc::new(schema), cols).expect("build batch");

    let err = COMMAND_LONG_DATA::from_record_batch(&batch).expect_err("expected decode error");
    let msg = err.to_string();
    assert!(
        msg.contains("param1") && msg.contains("null"),
        "unexpected error: {msg}"
    );
}

/// Sanity check that the helper batch the negative tests build also
/// decodes cleanly, so a regression in those tests' fixture is
/// distinguishable from a regression in the validation paths.
#[test]
fn accepts_well_formed_batch() {
    let schema = COMMAND_LONG_DATA::schema();
    let batch =
        RecordBatch::try_new(Arc::new(schema), good_command_long_columns()).expect("build batch");
    let decoded = COMMAND_LONG_DATA::from_record_batch(&batch).expect("decode");
    assert_eq!(decoded.target_system, 1);
    assert_eq!(decoded.confirmation, 0);
}
