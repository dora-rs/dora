use crate::{
  dds::qos::QosPolicies,
  discovery::sedp_messages::TopicBuiltinTopicData,
  security::{authentication::*, *},
};
use super::*;

/// Access control plugin interface: section 8.4.2.9 of the Security
/// specification (v. 1.1).
///
/// To make use of Rust's features, the trait functions deviate a bit from the
/// specification. The main difference is that the functions return a Result
/// type. With this, there is no need to provide a pointer to a
/// SecurityException type which the function would fill in case of a failure.
/// Instead, the Err-variant of the result contains the error information. Also,
/// if a function has a single return value, it is returned inside the
/// Ok-variant. When a function returns a boolean according to the
/// specification, the Ok-variant is interpreted as true and Err-variant as
/// false.
pub trait AccessControl {
  /// validate_local_permissions: section 8.4.2.9.1 of the Security
  /// specification
  fn validate_local_permissions(
    &self,
    auth_plugin: &impl Authentication,
    identity: IdentityHandle,
    domain_id: u16,
    participant_qos: &QosPolicies,
  ) -> SecurityResult<PermissionsHandle>;

  /// validate_remote_permissions: section 8.4.2.9.2 of the Security
  /// specification
  fn validate_remote_permissions(
    &self,
    auth_plugin: &impl Authentication,
    local_identity_handle: IdentityHandle,
    remote_identity_handle: IdentityHandle,
    remote_permissions_token: PermissionsToken,
    remote_credential_token: AuthenticatedPeerCredentialToken,
  ) -> SecurityResult<PermissionsHandle>;

  /// check_create_participant: section 8.4.2.9.3 of the Security
  /// specification
  fn check_create_participant(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    qos: &QosPolicies,
  ) -> SecurityResult<()>;

  /// check_create_datawriter: section 8.4.2.9.4 of the Security
  /// specification. The parameters partition and data_tag have been left out,
  /// since RustDDS does not yet support PartitionQoS or data tagging
  fn check_create_datawriter(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_name: String,
    qos: &QosPolicies,
  ) -> SecurityResult<()>;

  /// check_create_datareader: section 8.4.2.9.5 of the Security
  /// specification. The parameters partition and data_tag have been left out,
  /// since RustDDS does not yet support PartitionQoS or data tagging
  fn check_create_datareader(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_name: String,
    qos: &QosPolicies,
  ) -> SecurityResult<()>;

  /// check_create_topic: section 8.4.2.9.6 of the Security
  /// specification
  fn check_create_topic(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_name: String,
    qos: &QosPolicies,
  ) -> SecurityResult<()>;

  /// check_local_datawriter_register_instance: section 8.4.2.9.7 of the
  /// Security specification.
  /// The function signature is not complete yet.
  fn check_local_datawriter_register_instance(
    &self,
    permissions_handle: PermissionsHandle,
    writer_todo: (),
    key_todo: (),
  ) -> SecurityResult<()>;

  /// check_local_datawriter_register_instance: section 8.4.2.9.8 of the
  /// Security specification.
  /// The function signature is not complete yet.
  fn check_local_datawriter_dispose_instance(
    &self,
    permissions_handle: PermissionsHandle,
    writer_todo: (),
    key_todo: (),
  ) -> SecurityResult<()>;

  /// check_remote_participant: section 8.4.2.9.9 of the Security
  /// specification.
  fn check_remote_participant(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    participant_data: &ParticipantBuiltinTopicDataSecure,
  ) -> SecurityResult<()>;

  /// check_remote_datawriter: section 8.4.2.9.10 of the Security
  /// specification.
  fn check_remote_datawriter(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    publication_data: &PublicationBuiltinTopicDataSecure,
  ) -> SecurityResult<()>;

  /// check_remote_datareader: section 8.4.2.9.11 of the Security
  /// specification.
  fn check_remote_datareader(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    subscription_data: &SubscriptionBuiltinTopicDataSecure,
    relay_only: &mut bool,
  ) -> SecurityResult<()>;

  /// check_remote_topic: section 8.4.2.9.12 of the Security
  /// specification.
  fn check_remote_topic(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_data: &TopicBuiltinTopicData,
  ) -> SecurityResult<()>;

  /// check_local_datawriter_match: section 8.4.2.9.13 of the Security
  /// specification.
  fn check_local_datawriter_match(
    &self,
    writer_permissions_handle: PermissionsHandle,
    reader_permissions_handle: PermissionsHandle,
    publication_data: &PublicationBuiltinTopicDataSecure,
    subscription_data: &SubscriptionBuiltinTopicDataSecure,
  ) -> SecurityResult<()>;

  /// check_local_datareader_match: section 8.4.2.9.14 of the Security
  /// specification.
  /// The parameter subscriber_partition is omitted since RustDDS does not yet
  /// support PartitionQoS.
  fn check_local_datareader_match(
    &self,
    reader_permissions_handle: PermissionsHandle,
    writer_permissions_handle: PermissionsHandle,
    subscription_data: &SubscriptionBuiltinTopicDataSecure,
    publication_data: &PublicationBuiltinTopicDataSecure,
  ) -> SecurityResult<()>;

  /// check_remote_datawriter_register_instance: section 8.4.2.9.15 of the
  /// Security specification.
  /// TODO: The function signature is not complete yet.
  fn check_remote_datawriter_register_instance(
    &self,
    permissions_handle: PermissionsHandle,
    reader_todo: (),
    publication_handle_todo: (),
    key_todo: (),
    instance_handle_todo: (),
  ) -> SecurityResult<()>;

  /// check_remote_datawriter_dispose_instance: section 8.4.2.9.16 of the
  /// Security specification.
  /// TODO: The function signature is not complete yet.
  fn check_remote_datawriter_dispose_instance(
    &self,
    permissions_handle: PermissionsHandle,
    reader_todo: (),
    publication_handle_todo: (),
    key_todo: (),
  ) -> SecurityResult<()>;

  /// get_permissions_token: section 8.4.2.9.17 of the Security
  /// specification.
  fn get_permissions_token(&self, handle: PermissionsHandle) -> SecurityResult<PermissionsToken>;

  /// get_permissions_credential_token: section 8.4.2.9.18 of the Security
  /// specification.
  fn get_permissions_credential_token(
    &self,
    handle: PermissionsHandle,
  ) -> SecurityResult<PermissionsCredentialToken>;

  /// set_listener: section 8.4.2.9.19 of the Security
  /// specification.
  /// TODO: we do not need this as listeners are not used in RustDDS, but which
  /// async mechanism to use?
  fn set_listener(&self) -> SecurityResult<()>;

  /// get_participant_sec_attributes: section 8.4.2.9.22 of the Security
  /// specification.
  fn get_participant_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
  ) -> SecurityResult<ParticipantSecurityAttributes>;

  /// get_topic_sec_attributes: section 8.4.2.9.23 of the Security
  /// specification.
  fn get_topic_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
    topic_name: String,
  ) -> SecurityResult<TopicSecurityAttributes>;

  /// get_datawriter_sec_attributes: section 8.4.2.9.24 of the Security
  /// specification.
  /// The parameters partition and data_tag have been left out,
  /// since RustDDS does not yet support PartitionQoS or data tagging
  fn get_datawriter_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
    topic_name: String,
  ) -> SecurityResult<EndpointSecurityAttributes>;

  /// get_datareader_sec_attributes: section 8.4.2.9.25 of the Security
  /// specification.
  /// The parameters partition and data_tag have been left out,
  /// since RustDDS does not yet support PartitionQoS or data tagging
  fn get_datareader_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
    topic_name: String,
  ) -> SecurityResult<EndpointSecurityAttributes>;

  // TODO: Can the different return methods (e.g. return_permissions_token) be
  // left out, since Rust manages memory for us?
}
