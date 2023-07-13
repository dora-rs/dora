// This is used for message parsing - validity checks.

// Having a separate trait is probably a bit overengineering.
pub trait Validity {
  fn valid(&self) -> bool;
}
