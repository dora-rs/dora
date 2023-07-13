use crate::{
  dds::qos::QosPolicies,
  security::{access_control::*, SecurityResult},
  structure::guid::GUID,
};
use super::*;

/// Authentication plugin interface: section 8.3.2.11 of the Security
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
pub trait Authentication {
  /// validate_local_identity: section 8.3.2.11.2 of the Security
  /// specification
  fn validate_local_identity(
    &mut self,
    local_identity_handle: &mut IdentityHandle,
    adjusted_participant_guid: &mut GUID,
    domain_id: u16,
    participant_qos: &QosPolicies,
    candidate_participant_guid: GUID,
  ) -> SecurityResult<ValidationOutcome>;

  /// validate_remote_identity: section 8.3.2.11.3 of the Security
  /// specification
  fn validate_remote_identity(
    &self,
    remote_identity_handle: &mut IdentityHandle,
    local_auth_request_token: &mut AuthRequestMessageToken,
    remote_auth_request_token: AuthRequestMessageToken,
    local_identity_handle: IdentityHandle,
    remote_identity_token: IdentityToken,
    remote_participant_guid: GUID,
  ) -> SecurityResult<ValidationOutcome>;

  /// begin_handshake_request: section 8.3.2.11.4 of the Security
  /// specification
  fn begin_handshake_request(
    &self,
    handshake_handle: &mut HandshakeHandle,
    handshake_message: &mut HandshakeMessageToken,
    initiator_identity_handle: IdentityHandle,
    replier_identity_handle: IdentityHandle,
    serialized_local_participant_data: Vec<u8>,
  ) -> SecurityResult<ValidationOutcome>;

  /// begin_handshake_reply: section 8.3.2.11.5 of the Security
  /// specification
  fn begin_handshake_reply(
    &self,
    handshake_handle: &mut HandshakeHandle,
    handshake_message_out: &mut HandshakeMessageToken,
    handshake_message_in: HandshakeMessageToken,
    initiator_identity_handle: IdentityHandle,
    replier_identity_handle: IdentityHandle,
    serialized_local_participant_data: Vec<u8>,
  ) -> SecurityResult<ValidationOutcome>;

  /// process_handshake: section 8.3.2.11.6 of the Security
  /// specification
  fn process_handshake(
    &self,
    handshake_message_out: &mut HandshakeMessageToken,
    handshake_message_in: HandshakeMessageToken,
    handshake_handle: HandshakeHandle,
  ) -> SecurityResult<ValidationOutcome>;

  /// get_shared_secret: section 8.3.2.11.7 of the Security
  /// specification
  fn get_shared_secret(
    &self,
    handshake_handle: HandshakeHandle,
  ) -> SecurityResult<SharedSecretHandle>;

  /// get_authenticated_peer_credential_token: section 8.3.2.11.8 of the
  /// Security specification
  fn get_authenticated_peer_credential_token(
    &self,
    handshake_handle: HandshakeHandle,
  ) -> SecurityResult<AuthenticatedPeerCredentialToken>;

  /// get_identity_token: section 8.3.2.11.9 of the Security
  /// specification
  fn get_identity_token(&self, handle: IdentityHandle) -> SecurityResult<IdentityToken>;

  /// get_identity_token: section 8.3.2.11.10 of the Security
  /// specification
  fn get_identity_status_token(
    &self,
    handle: IdentityHandle,
  ) -> SecurityResult<IdentityStatusToken>;

  /// set_permissions_credential_and_token: section 8.3.2.11.11 of the Security
  /// specification
  fn set_permissions_credential_and_token(
    &self,
    handle: IdentityHandle,
    permissions_credential_token: PermissionsCredentialToken,
    permissions_token: PermissionsToken,
  ) -> SecurityResult<()>;

  /// set_listener: section 8.3.2.11.12 of the Security
  /// specification.
  /// TODO: we do not need this as listeners are not used in RustDDS, but which
  /// async mechanism to use?
  fn set_listener(&self) -> SecurityResult<()>;

  // TODO: Can the different return methods (e.g. return_identity_token) be left
  // out, since Rust manages memory for us?
}
