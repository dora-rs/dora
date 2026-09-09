use crate::{DoraArray, IntoArrow, internal::from_array_ref};
use arrow::array::{PrimitiveArray, StringArray, TimestampNanosecondArray};
use arrow::datatypes::{
    ArrowTimestampType, Float16Type, Float32Type, Float64Type, Int8Type, Int16Type, Int32Type,
    Int64Type, UInt8Type, UInt16Type, UInt32Type, UInt64Type,
};
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use half::f16;
use tracing::warn;

/// Wrap a concrete Arrow array as a [`DoraArray`].
fn wrap(array: impl arrow::array::Array + 'static) -> DoraArray {
    from_array_ref(std::sync::Arc::new(array))
}

impl IntoArrow for bool {
    fn into_arrow(self) -> DoraArray {
        wrap(std::iter::once(Some(self)).collect::<arrow::array::BooleanArray>())
    }
}

macro_rules! impl_into_arrow {
    ($($t:ty => $arrow_type:ty),*) => {
        $(
            impl IntoArrow for $t {
                fn into_arrow(self) -> DoraArray {
                    wrap(std::iter::once(self).collect::<PrimitiveArray<$arrow_type>>())
                }
            }
        )*
        $(
            impl IntoArrow for Vec<$t> {
                fn into_arrow(self) -> DoraArray {
                    wrap(PrimitiveArray::<$arrow_type>::from(self))
                }
            }
        )*
    };
}

impl_into_arrow!(
    u8 => UInt8Type,
    u16 => UInt16Type,
    u32 => UInt32Type,
    u64 => UInt64Type,
    i8 => Int8Type,
    i16 => Int16Type,
    i32 => Int32Type,
    i64 => Int64Type,
    f16 => Float16Type,
    f32 => Float32Type,
    f64 => Float64Type
);

impl IntoArrow for &str {
    fn into_arrow(self) -> DoraArray {
        wrap(std::iter::once(Some(self)).collect::<StringArray>())
    }
}

impl IntoArrow for () {
    fn into_arrow(self) -> DoraArray {
        wrap(arrow::array::NullArray::new(0))
    }
}

impl IntoArrow for NaiveDate {
    fn into_arrow(self) -> DoraArray {
        wrap(arrow::array::Date64Array::from(vec![
            arrow::datatypes::Date64Type::from_naive_date(self),
        ]))
    }
}

impl IntoArrow for NaiveTime {
    fn into_arrow(self) -> DoraArray {
        wrap(arrow::array::Time64NanosecondArray::from(vec![
            arrow::array::temporal_conversions::time_to_time64ns(self),
        ]))
    }
}

impl IntoArrow for String {
    fn into_arrow(self) -> DoraArray {
        wrap(std::iter::once(Some(self)).collect::<StringArray>())
    }
}

impl IntoArrow for Vec<String> {
    fn into_arrow(self) -> DoraArray {
        wrap(StringArray::from(self))
    }
}

/// The nanosecond-resolution `i64` timestamp can only represent dates in roughly
/// 1677-09-21..2262-04-11. Dates outside that range are saturated to `i64::MIN`
/// (far-past) or `i64::MAX` (far-future) and a `tracing::warn!` is emitted,
/// rather than silently mapping to the Unix epoch (the previous behaviour).
impl IntoArrow for NaiveDateTime {
    fn into_arrow(self) -> DoraArray {
        let timestamp =
            match arrow::datatypes::TimestampNanosecondType::from_naive_datetime(self, None) {
                Some(ts) => ts,
                None => {
                    let epoch = chrono::DateTime::UNIX_EPOCH.naive_utc();
                    let saturated = if self >= epoch { i64::MAX } else { i64::MIN };
                    warn!(
                        datetime = %self,
                        saturated_ns = saturated,
                        "NaiveDateTime is outside the nanosecond-representable range \
                         (~1677-09-21..2262-04-11); saturating to boundary value. \
                         Consider using a timestamp type with a wider or lower-resolution range."
                    );
                    saturated
                }
            };
        wrap(TimestampNanosecondArray::from(vec![timestamp]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::Array as _;
    use chrono::NaiveDate;

    fn timestamp_of(data: &DoraArray) -> i64 {
        crate::internal::array_ref(data)
            .as_any()
            .downcast_ref::<TimestampNanosecondArray>()
            .expect("timestamp array")
            .value(0)
    }

    #[test]
    fn naive_datetime_out_of_range_saturates() {
        // Far-future date (year 3000) should saturate to i64::MAX
        let far_future = NaiveDate::from_ymd_opt(3000, 1, 1)
            .unwrap()
            .and_hms_opt(0, 0, 0)
            .unwrap();
        let arr = far_future.into_arrow();
        assert_eq!(timestamp_of(&arr), i64::MAX);

        // Far-past date (year 1000) should saturate to i64::MIN
        let far_past = NaiveDate::from_ymd_opt(1000, 1, 1)
            .unwrap()
            .and_hms_opt(0, 0, 0)
            .unwrap();
        let arr = far_past.into_arrow();
        assert_eq!(timestamp_of(&arr), i64::MIN);
    }
}

/// `IntoArrow` for Arrow arrays of dora's **internal** major, so
/// `node.send_output(id, params, my_arrow_array)` keeps working for callers on
/// that major with no wrapping and no conversion.
///
/// Gated for the same reason [`DoraArray::as_array`](crate::DoraArray::as_array)
/// is: these impls name a specific Arrow major. When dora moves internally to
/// Arrow 60 they re-gate behind `arrow-v60`, and `arrow-v59` keeps a converting
/// equivalent — visibly, rather than silently changing meaning under callers.
///
/// A blanket `impl<A: arrow::array::Array> IntoArrow for A` is not possible:
/// it overlaps with the primitive impls above under coherence's
/// "upstream crates may add a new impl" rule, so the concrete types are listed.
#[cfg(feature = "arrow-v59")]
mod arrow_v59_impls {
    use super::{DoraArray, IntoArrow, from_array_ref};
    use arrow::array::{ArrayRef, PrimitiveArray};
    use arrow::datatypes::ArrowPrimitiveType;

    impl IntoArrow for ArrayRef {
        fn into_arrow(self) -> DoraArray {
            from_array_ref(self)
        }
    }

    impl<T: ArrowPrimitiveType> IntoArrow for PrimitiveArray<T> {
        fn into_arrow(self) -> DoraArray {
            from_array_ref(std::sync::Arc::new(self))
        }
    }

    macro_rules! impl_into_arrow_for_arrow_array {
        ($($t:ty),* $(,)?) => {
            $(
                impl IntoArrow for $t {
                    fn into_arrow(self) -> DoraArray {
                        from_array_ref(std::sync::Arc::new(self))
                    }
                }
            )*
        };
    }

    impl_into_arrow_for_arrow_array!(
        arrow::array::BooleanArray,
        arrow::array::NullArray,
        arrow::array::StringArray,
        arrow::array::LargeStringArray,
        arrow::array::StringViewArray,
        arrow::array::BinaryArray,
        arrow::array::LargeBinaryArray,
        arrow::array::BinaryViewArray,
        arrow::array::FixedSizeBinaryArray,
        arrow::array::StructArray,
        arrow::array::ListArray,
        arrow::array::LargeListArray,
        arrow::array::FixedSizeListArray,
        arrow::array::MapArray,
        arrow::array::UnionArray,
    );
}
