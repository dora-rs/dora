#[macro_export]
macro_rules! define_enum_from {
    ($into_t:ty, $from_t:ty, $path:path) => {
        impl From<$from_t> for $into_t {
            fn from(t: $from_t) -> Self {
                $path(t)
            }
        }
    };
}
