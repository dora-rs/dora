/// Description of the type of a [Topic](../struct.Topic.html)
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct TypeDesc {
  my_name: String, // this is a rather minimal implementation
} // placeholders

impl TypeDesc {
  pub fn new(my_name: String) -> Self {
    Self { my_name }
  }
  pub fn name(&self) -> &str {
    &self.my_name
  }
}
