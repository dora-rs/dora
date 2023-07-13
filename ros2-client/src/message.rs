use serde::{de::DeserializeOwned, Serialize};

/// Trait to ensure Messages can be (de)serialized
pub trait Message: Serialize + DeserializeOwned {}

impl Message for () {}
impl Message for String {}

impl Message for i8 {}
impl Message for i16 {}
impl Message for i32 {}
impl Message for i64 {}

impl Message for u8 {}
impl Message for u16 {}
impl Message for u32 {}
impl Message for u64 {}

impl<T: Message> Message for Vec<T> {}

pub struct MessageTypeName {
  //TODO: String is UTF-8, but ROS2 uses just ASCII
  ros2_package_name: String, // or shoudl theis ne "namespace"?
  ros2_type_name: String,
}

impl MessageTypeName {
  pub fn new(package_name: &str, type_name: &str) -> Self {
    //TODO: Ensure parameters have no leading/trailing slashes
    MessageTypeName {
      ros2_package_name: package_name.to_owned(),
      ros2_type_name: type_name.to_owned(),
    }
  }

  pub fn dds_msg_type(&self) -> String {
    slash_to_colons(self.ros2_package_name.clone() + "/msg/dds_/" + &self.ros2_type_name + "_")
  }

  pub fn dds_request_type(&self) -> String {
    slash_to_colons(
      self.ros2_package_name.clone() + "/srv/dds_/" + &self.ros2_type_name + "_Request_",
    )
  }

  pub fn dds_response_type(&self) -> String {
    slash_to_colons(
      self.ros2_package_name.clone() + "/srv/dds_/" + &self.ros2_type_name + "_Response_",
    )
  }

  pub fn dds_action_type(&self) -> String {
    slash_to_colons(self.ros2_package_name.clone() + "/action/dds_/" + &self.ros2_type_name)
  }
}

fn slash_to_colons(s: String) -> String {
  s.replace('/', "::")
}
