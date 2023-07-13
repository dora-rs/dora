use crate::{
  dds::qos::QosPolicies,
  security::{access_control::*, SecurityResult},
  structure::guid::GUID,
};
use super::*;

// A struct implementing the built-in Authentication plugin
// See sections 8.3 and 9.3 of the Security specification (v. 1.1)
pub struct AuthenticationBuiltIn {
  todo: String,
}

impl Authentication for AuthenticationBuiltIn {
  // Currently only mocked
  fn validate_local_identity(
    &mut self,
    local_identity_handle: &mut IdentityHandle,
    adjusted_participant_guid: &mut GUID,
    domain_id: u16,
    participant_qos: &QosPolicies,
    candidate_participant_guid: GUID,
  ) -> SecurityResult<ValidationOutcome> {
    // TODO: actual implementation

    *local_identity_handle = IdentityHandle::MOCK;
    *adjusted_participant_guid = candidate_participant_guid;

    Ok(ValidationOutcome::Ok)
  }

  // Currently only mocked
  fn get_identity_token(&self, handle: IdentityHandle) -> SecurityResult<IdentityToken> {
    // TODO: actual implementation

    Ok(IdentityToken::MOCK)
  }

  // Currently only mocked
  fn get_identity_status_token(
    &self,
    handle: IdentityHandle,
  ) -> SecurityResult<IdentityStatusToken> {
    // TODO: actual implementation

    Ok(IdentityStatusToken::MOCK)
  }

  // Currently only mocked
  fn set_permissions_credential_and_token(
    &self,
    handle: IdentityHandle,
    permissions_credential_token: PermissionsCredentialToken,
    permissions_token: PermissionsToken,
  ) -> SecurityResult<()> {
    // TODO: actual implementation

    Ok(())
  }

  // Currently only mocked
  fn validate_remote_identity(
    &self,
    remote_identity_handle: &mut IdentityHandle,
    local_auth_request_token: &mut AuthRequestMessageToken,
    remote_auth_request_token: AuthRequestMessageToken,
    local_identity_handle: IdentityHandle,
    remote_identity_token: IdentityToken,
    remote_participant_guid: GUID,
  ) -> SecurityResult<ValidationOutcome> {
    // TODO: actual implementation
    *remote_identity_handle = IdentityHandle::MOCK;
    *local_auth_request_token = AuthRequestMessageToken::MOCK;

    Ok(ValidationOutcome::Ok)
  }

  fn begin_handshake_request(
    &self,
    handshake_handle: &mut HandshakeHandle,
    handshake_message: &mut HandshakeMessageToken,
    initiator_identity_handle: IdentityHandle,
    replier_identity_handle: IdentityHandle,
    serialized_local_participant_data: Vec<u8>,
  ) -> SecurityResult<ValidationOutcome> {
    todo!();
  }

  fn begin_handshake_reply(
    &self,
    handshake_handle: &mut HandshakeHandle,
    handshake_message_out: &mut HandshakeMessageToken,
    handshake_message_in: HandshakeMessageToken,
    initiator_identity_handle: IdentityHandle,
    replier_identity_handle: IdentityHandle,
    serialized_local_participant_data: Vec<u8>,
  ) -> SecurityResult<ValidationOutcome> {
    todo!();
  }

  fn process_handshake(
    &self,
    handshake_message_out: &mut HandshakeMessageToken,
    handshake_message_in: HandshakeMessageToken,
    handshake_handle: HandshakeHandle,
  ) -> SecurityResult<ValidationOutcome> {
    todo!();
  }

  // Currently only mocked
  fn get_shared_secret(
    &self,
    handshake_handle: HandshakeHandle,
  ) -> SecurityResult<SharedSecretHandle> {
    // TODO: actual implementation

    Ok(SharedSecretHandle::MOCK)
  }

  // Currently only mocked
  fn get_authenticated_peer_credential_token(
    &self,
    handshake_handle: HandshakeHandle,
  ) -> SecurityResult<AuthenticatedPeerCredentialToken> {
    // TODO: actual implementation

    Ok(AuthenticatedPeerCredentialToken::MOCK)
  }

  fn set_listener(&self) -> SecurityResult<()> {
    todo!();
  }
}
