use crate::IntoArrow;
use arrow::array::{PrimitiveArray, StringArray, TimestampNanosecondArray};
use arrow::datatypes::{
    ArrowPrimitiveType, ArrowTimestampType, Float16Type, Float32Type, Float64Type, Int16Type,
    Int32Type, Int64Type, Int8Type, UInt16Type, UInt32Type, UInt64Type, UInt8Type,
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

impl IntoArrow for NaiveDateTime {
    type A = arrow::array::TimestampNanosecondArray;
    fn into_arrow(self) -> Self::A {
        let timestamp = match arrow::datatypes::TimestampNanosecondType::make_value(self) {
            Some(timestamp) => timestamp,
            None => arrow::datatypes::TimestampNanosecondType::default_value(),
        };
        TimestampNanosecondArray::from(vec![timestamp])
    }
}
