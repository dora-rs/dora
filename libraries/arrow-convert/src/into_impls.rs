use crate::IntoArrow;
use tracing::warn;
use arrow::array::{PrimitiveArray, StringArray, TimestampNanosecondArray};
use arrow::datatypes::{
    ArrowTimestampType, Float16Type, Float32Type, Float64Type, Int8Type,
    Int16Type, Int32Type, Int64Type, UInt8Type, UInt16Type, UInt32Type, UInt64Type,
};
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};
use half::f16;

impl IntoArrow for bool {
    type A = arrow::array::BooleanArray;
    fn into_arrow(self) -> Self::A {
        std::iter::once(Some(self)).collect()
    }
}

macro_rules! impl_into_arrow {
    ($($t:ty => $arrow_type:ty),*) => {
        $(
            impl IntoArrow for $t {
                type A = PrimitiveArray<$arrow_type>;
                fn into_arrow(self) -> Self::A {
                    std::iter::once(self).collect()
                }
            }
        )*

        $(
            impl IntoArrow for Vec<$t> {
                type A = PrimitiveArray<$arrow_type>;
                fn into_arrow(self) -> Self::A {
                    self.into()
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
    type A = StringArray;
    fn into_arrow(self) -> Self::A {
        std::iter::once(Some(self)).collect()
    }
}

impl IntoArrow for () {
    type A = arrow::array::NullArray;

    fn into_arrow(self) -> Self::A {
        arrow::array::NullArray::new(0)
    }
}

impl IntoArrow for NaiveDate {
    type A = arrow::array::Date64Array;
    fn into_arrow(self) -> Self::A {
        arrow::array::Date64Array::from(vec![arrow::datatypes::Date64Type::from_naive_date(self)])
    }
}

impl IntoArrow for NaiveTime {
    type A = arrow::array::Time64NanosecondArray;
    fn into_arrow(self) -> Self::A {
        arrow::array::Time64NanosecondArray::from(vec![
            arrow::array::temporal_conversions::time_to_time64ns(self),
        ])
    }
}

impl IntoArrow for String {
    type A = StringArray;
    fn into_arrow(self) -> Self::A {
        std::iter::once(Some(self)).collect()
    }
}

impl IntoArrow for Vec<String> {
    type A = StringArray;
    fn into_arrow(self) -> Self::A {
        StringArray::from(self)
    }
}

/// The nanosecond-resolution `i64` timestamp can only represent dates in roughly
/// representable boundary and a warning is emitted, rather than silently mapping
/// to the Unix epoch (the previous behaviour).
impl IntoArrow for NaiveDateTime {
    type A = arrow::array::TimestampNanosecondArray;
    fn into_arrow(self) -> Self::A {
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
        TimestampNanosecondArray::from(vec![timestamp])
    }
}
