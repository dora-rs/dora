use crate::IntoArrow;
use arrow::array::{Array, ArrayRef, PrimitiveArray, StringArray};
use arrow_convert::serialize::TryIntoArrow;

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

impl IntoArrow for &String {
    type A = StringArray;

    fn into_arrow(self) -> Self::A {
        match vec![self.clone()].try_into_arrow() {
            Ok(array_ref) => {
                let array_ref: ArrayRef = array_ref; // Ensuring explicit type annotation
                let array: &dyn Array = array_ref.as_ref(); // Dereference Arc<dyn Array>

                if let Some(string_array) = array.as_any().downcast_ref::<StringArray>() {
                    string_array.clone()
                } else {
                    eprintln!("Failed to downcast to StringArray.");
                    StringArray::from(vec![""]) // Fallback in case of failure
                }
            }
            Err(err) => {
                eprintln!("Failed to Create String Array: {}", err);
                StringArray::from(vec![""]) // Safe fallback
            }
        }
    }
}
