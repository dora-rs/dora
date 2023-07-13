use crate::{
  dds::qos::QosPolicies,
  discovery::sedp_messages::TopicBuiltinTopicData,
  security::{authentication::*, *},
};
use super::*;

// A struct implementing the built-in Access control plugin
// See sections 8.4 and 9.4 of the Security specification (v. 1.1)
pub struct AccessControlBuiltIn {
  todo: String,
}

impl AccessControl for AccessControlBuiltIn {
  // Currently only mocked
  fn validate_local_permissions(
    &self,
    auth_plugin: &impl Authentication,
    identity: IdentityHandle,
    domain_id: u16,
    participant_qos: &QosPolicies,
  ) -> SecurityResult<PermissionsHandle> {
    // TODO: actual implementation

    Ok(PermissionsHandle::MOCK)
  }

  // Currently only mocked
  fn validate_remote_permissions(
    &self,
    auth_plugin: &impl Authentication,
    local_identity_handle: IdentityHandle,
    remote_identity_handle: IdentityHandle,
    remote_permissions_token: PermissionsToken,
    remote_credential_token: AuthenticatedPeerCredentialToken,
  ) -> SecurityResult<PermissionsHandle> {
    // TODO: actual implementation

    Ok(PermissionsHandle::MOCK)
  }

  // Currently only mocked
  fn check_create_participant(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    qos: &QosPolicies,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_create_datawriter(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_name: String,
    qos: &QosPolicies,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_create_datareader(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_name: String,
    qos: &QosPolicies,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_create_topic(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_name: String,
    qos: &QosPolicies,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  fn check_local_datawriter_register_instance(
    &self,
    permissions_handle: PermissionsHandle,
    writer_todo: (),
    key_todo: (),
  ) -> SecurityResult<()> {
    todo!();
  }

  fn check_local_datawriter_dispose_instance(
    &self,
    permissions_handle: PermissionsHandle,
    writer_todo: (),
    key_todo: (),
  ) -> SecurityResult<()> {
    todo!();
  }

  // Currently only mocked
  fn check_remote_participant(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    participant_data: &ParticipantBuiltinTopicDataSecure,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_remote_datawriter(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    publication_data: &PublicationBuiltinTopicDataSecure,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_remote_datareader(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    subscription_data: &SubscriptionBuiltinTopicDataSecure,
    relay_only: &mut bool,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_remote_topic(
    &self,
    permissions_handle: PermissionsHandle,
    domain_id: u16,
    topic_data: &TopicBuiltinTopicData,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_local_datawriter_match(
    &self,
    writer_permissions_handle: PermissionsHandle,
    reader_permissions_handle: PermissionsHandle,
    publication_data: &PublicationBuiltinTopicDataSecure,
    subscription_data: &SubscriptionBuiltinTopicDataSecure,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn check_local_datareader_match(
    &self,
    reader_permissions_handle: PermissionsHandle,
    writer_permissions_handle: PermissionsHandle,
    subscription_data: &SubscriptionBuiltinTopicDataSecure,
    publication_data: &PublicationBuiltinTopicDataSecure,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  fn check_remote_datawriter_register_instance(
    &self,
    permissions_handle: PermissionsHandle,
    reader_todo: (),
    publication_handle_todo: (),
    key_todo: (),
    instance_handle_todo: (),
  ) -> SecurityResult<()> {
    todo!();
  }

  fn check_remote_datawriter_dispose_instance(
    &self,
    permissions_handle: PermissionsHandle,
    reader_todo: (),
    publication_handle_todo: (),
    key_todo: (),
  ) -> SecurityResult<()> {
    todo!();
  }

  // Currently only mocked
  fn get_permissions_token(&self, handle: PermissionsHandle) -> SecurityResult<PermissionsToken> {
    // TODO: actual implementation

    Ok(PermissionsToken::MOCK)
  }

  // Currently only mocked
  fn get_permissions_credential_token(
    &self,
    handle: PermissionsHandle,
  ) -> SecurityResult<PermissionsCredentialToken> {
    // TODO: actual implementation

    Ok(PermissionsCredentialToken::MOCK)
  }

  fn set_listener(&self) -> SecurityResult<()> {
    todo!();
  }

  // Currently only mocked
  fn get_participant_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
  ) -> SecurityResult<ParticipantSecurityAttributes> {
    // TODO: actual implementation

    Ok(ParticipantSecurityAttributes::MOCK)
  }

  // Currently only mocked
  fn get_topic_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
    topic_name: String,
  ) -> SecurityResult<TopicSecurityAttributes> {
    // TODO: actual implementation

    Ok(TopicSecurityAttributes::MOCK)
  }

  // Currently only mocked
  fn get_datawriter_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
    topic_name: String,
  ) -> SecurityResult<EndpointSecurityAttributes> {
    // TODO: actual implementation

    Ok(EndpointSecurityAttributes::MOCK)
  }

  // Currently only mocked
  fn get_datareader_sec_attributes(
    &self,
    permissions_handle: PermissionsHandle,
    topic_name: String,
  ) -> SecurityResult<EndpointSecurityAttributes> {
    // TODO: actual implementation

    Ok(EndpointSecurityAttributes::MOCK)
  }
}
