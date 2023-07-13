// This macro is used to create checked arithmetic operator implementations
// for custom number types.
macro_rules! checked_impl {
  ($trait_name:ident, $method:ident, $t:ty) => {
    use num_traits::$trait_name;

    impl $trait_name for $t {
      #[inline]
      fn $method(&self, v: &Self) -> Option<Self> {
        (self.0).$method(v.0).map(Self::from)
      }
    }
  };
}
