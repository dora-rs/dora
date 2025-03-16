use crate::IntoArrow;
use arrow::array::{PrimitiveArray, StringArray, TimestampNanosecondArray};
use arrow::datatypes::{ArrowPrimitiveType, ArrowTimestampType};
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};

impl IntoArrow for bool {
    type A = arrow::array::BooleanArray;
    fn into_arrow(self) -> Self::A {
        std::iter::once(Some(self)).collect()
    }
}

impl IntoArrow for u8 {
    type A = PrimitiveArray<arrow::datatypes::UInt8Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for u16 {
    type A = PrimitiveArray<arrow::datatypes::UInt16Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for u32 {
    type A = PrimitiveArray<arrow::datatypes::UInt32Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for u64 {
    type A = PrimitiveArray<arrow::datatypes::UInt64Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for i8 {
    type A = PrimitiveArray<arrow::datatypes::Int8Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for i16 {
    type A = PrimitiveArray<arrow::datatypes::Int16Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for i32 {
    type A = PrimitiveArray<arrow::datatypes::Int32Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for i64 {
    type A = PrimitiveArray<arrow::datatypes::Int64Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for f32 {
    type A = PrimitiveArray<arrow::datatypes::Float32Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}
impl IntoArrow for f64 {
    type A = PrimitiveArray<arrow::datatypes::Float64Type>;
    fn into_arrow(self) -> Self::A {
        std::iter::once(self).collect()
    }
}

impl IntoArrow for &str {
    type A = StringArray;
    fn into_arrow(self) -> Self::A {
        std::iter::once(Some(self)).collect()
    }
}

impl IntoArrow for Vec<u8> {
    type A = PrimitiveArray<arrow::datatypes::UInt8Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<u16> {
    type A = PrimitiveArray<arrow::datatypes::UInt16Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<u32> {
    type A = PrimitiveArray<arrow::datatypes::UInt32Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<u64> {
    type A = PrimitiveArray<arrow::datatypes::UInt64Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<i8> {
    type A = PrimitiveArray<arrow::datatypes::Int8Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<i16> {
    type A = PrimitiveArray<arrow::datatypes::Int16Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<i32> {
    type A = PrimitiveArray<arrow::datatypes::Int32Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<i64> {
    type A = PrimitiveArray<arrow::datatypes::Int64Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<f32> {
    type A = PrimitiveArray<arrow::datatypes::Float32Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
    }
}
impl IntoArrow for Vec<f64> {
    type A = PrimitiveArray<arrow::datatypes::Float64Type>;
    fn into_arrow(self) -> Self::A {
        self.into()
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
