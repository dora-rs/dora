#[macro_export]
macro_rules! impl_from_trait_for_enum {
    (
        $new_type:ty,
        $old_type:ty,
        $(
            $(#[$m:meta])?
            $new_item:ident := $old_item:ident
        ),+
        $(,)?
    ) => {
        impl From<$old_type> for $new_type {
            fn from(item: $old_type) -> Self {
                match item {
                    $(
                        $(#[$m])?
                        <$old_type>::$old_item => Self::$new_item
                    ),+
                }
            }
        }

        impl From<$new_type> for $old_type {
            fn from(item: $new_type) -> Self {
                match item {
                    $(
                        $(#[$m])?
                        <$new_type>::$new_item => Self::$old_item
                    ),+
                }
            }
        }

        impl From<$new_type> for std::os::raw::c_int {
            fn from(item: $new_type) -> Self {
                <$old_type>::from(item) as Self
            }
        }
    };
}
