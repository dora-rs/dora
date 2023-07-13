use crate::{
  messages::submessages::elements::{
    crypto_content::CryptoContent, parameter_list::ParameterList,
    serialized_payload::SerializedPayload,
  },
  rtps::{Message, Submessage},
  security::{
    access_control::types::*,
    authentication::types::*,
    cryptographic::{cryptographic_plugin::*, types::*},
    types::*,
  },
};

// A struct implementing the built-in Cryptographic plugin
// See sections 8.5 and 9.5 of the Security specification (v. 1.1)
pub struct CryptographicBuiltIn {}

impl CryptoKeyFactory for CryptographicBuiltIn {
  fn register_local_participant(
    participant_identity: IdentityHandle,
    participant_permissions: PermissionsHandle,
    participant_properties: Vec<Property>,
    participant_security_attributes: ParticipantSecurityAttributes,
  ) -> SecurityResult<ParticipantCryptoHandle> {
    todo!();
  }

  fn register_matched_remote_participant(
    local_participant_crypto_handle: ParticipantCryptoHandle,
    remote_participant_identity: IdentityHandle,
    remote_participant_permissions: PermissionsHandle,
    shared_secret: SharedSecretHandle,
  ) -> SecurityResult<ParticipantCryptoHandle> {
    todo!();
  }

  fn register_local_datawriter(
    participant_crypto: ParticipantCryptoHandle,
    datawriter_properties: Vec<Property>,
    datawriter_security_attributes: EndpointSecurityAttributes,
  ) -> SecurityResult<DatawriterCryptoHandle> {
    todo!();
  }

  fn register_matched_remote_datareader(
    local_datawriter_crypto_handle: DatawriterCryptoHandle,
    remote_participant_crypto: ParticipantCryptoHandle,
    shared_secret: SharedSecretHandle,
    relay_only: bool,
  ) -> SecurityResult<DatareaderCryptoHandle> {
    todo!();
  }

  fn register_local_datareader(
    participant_crypto: ParticipantCryptoHandle,
    datareader_properties: Vec<Property>,
    datareader_security_attributes: EndpointSecurityAttributes,
  ) -> SecurityResult<DatareaderCryptoHandle> {
    todo!();
  }

  fn register_matched_remote_datawriter(
    local_datareader_crypto_handle: DatareaderCryptoHandle,
    remote_participant_crypt: ParticipantCryptoHandle,
    shared_secret: SharedSecretHandle,
  ) -> SecurityResult<DatareaderCryptoHandle> {
    todo!();
  }

  fn unregister_participant(
    participant_crypto_handle: ParticipantCryptoHandle,
  ) -> SecurityResult<()> {
    todo!();
  }

  fn unregister_datawriter(datawriter_crypto_handle: DatawriterCryptoHandle) -> SecurityResult<()> {
    todo!();
  }

  fn unregister_datareader(datareader_crypto_handle: DatareaderCryptoHandle) -> SecurityResult<()> {
    todo!();
  }
}

impl CryptoKeyExchange for CryptographicBuiltIn {
  fn create_local_participant_crypto_tokens(
    local_participant_crypto: ParticipantCryptoHandle,
    remote_participant_crypto: ParticipantCryptoHandle,
  ) -> SecurityResult<Vec<ParticipantCryptoToken>> {
    todo!();
  }

  fn set_remote_participant_crypto_tokens(
    local_participant_crypto: ParticipantCryptoHandle,
    remote_participant_crypto: ParticipantCryptoHandle,
    remote_participant_tokens: Vec<ParticipantCryptoToken>,
  ) -> SecurityResult<()> {
    todo!();
  }

  fn create_local_datawriter_crypto_tokens(
    local_datawriter_crypto: DatawriterCryptoHandle,
    remote_datareader_crypto: DatareaderCryptoHandle,
  ) -> SecurityResult<Vec<DatawriterCryptoToken>> {
    todo!();
  }

  fn set_remote_datawriter_crypto_tokens(
    local_datareader_crypto: DatareaderCryptoHandle,
    remote_datawriter_crypto: DatawriterCryptoHandle,
    remote_datawriter_tokens: Vec<DatawriterCryptoToken>,
  ) -> SecurityResult<()> {
    todo!();
  }

  fn create_local_datareader_crypto_tokens(
    local_datareader_crypto: DatareaderCryptoHandle,
    remote_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<Vec<DatareaderCryptoToken>> {
    todo!();
  }

  fn set_remote_datareader_crypto_tokens(
    local_datawriter_crypto: DatawriterCryptoHandle,
    remote_datareader_crypto: DatareaderCryptoHandle,
    remote_datareader_tokens: Vec<DatareaderCryptoToken>,
  ) -> SecurityResult<()> {
    todo!();
  }

  fn return_crypto_tokens(crypto_tokens: Vec<CryptoToken>) -> SecurityResult<()> {
    todo!();
  }
}

impl CryptoTransform for CryptographicBuiltIn {
  fn encode_serialized_payload(
    plain_buffer: SerializedPayload,
    sending_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<(CryptoContent, ParameterList)> {
    todo!();
  }

  fn encode_datawriter_submessage(
    plain_rtps_submessage: Submessage,
    sending_datawriter_crypto: DatawriterCryptoHandle,
    receiving_datareader_crypto_list: Vec<DatareaderCryptoHandle>,
  ) -> SecurityResult<EncodeResult<EncodedSubmessage>> {
    todo!();
  }

  fn encode_datareader_submessage(
    plain_rtps_submessage: Submessage,
    sending_datareader_crypto: DatareaderCryptoHandle,
    receiving_datawriter_crypto_list: Vec<DatawriterCryptoHandle>,
  ) -> SecurityResult<EncodeResult<EncodedSubmessage>> {
    todo!();
  }

  fn encode_rtps_message(
    plain_rtps_message: Message,
    sending_participant_crypto: ParticipantCryptoHandle,
    receiving_participant_crypto_list: Vec<ParticipantCryptoHandle>,
  ) -> SecurityResult<EncodeResult<Message>> {
    todo!();
  }

  fn decode_rtps_message(
    encoded_buffer: Message,
    receiving_participant_crypto: ParticipantCryptoHandle,
    sending_participant_crypto: ParticipantCryptoHandle,
  ) -> SecurityResult<Message> {
    todo!();
  }

  fn preprocess_secure_submsg(
    encoded_rtps_submessage: Submessage,
    receiving_participant_crypto: ParticipantCryptoHandle,
    sending_participant_crypto: ParticipantCryptoHandle,
  ) -> SecurityResult<SecureSubmessageCategory> {
    todo!();
  }

  fn decode_datawriter_submessage(
    encoded_rtps_submessage: (Submessage, Submessage, Submessage),
    receiving_datareader_crypto: DatareaderCryptoHandle,
    sending_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<Submessage> {
    todo!();
  }

  fn decode_datareader_submessage(
    encoded_rtps_submessage: (Submessage, Submessage, Submessage),
    receiving_datawriter_crypto: DatawriterCryptoHandle,
    sending_datareader_crypto: DatareaderCryptoHandle,
  ) -> SecurityResult<Submessage> {
    todo!();
  }

  fn decode_serialized_payload(
    encoded_buffer: CryptoContent,
    inline_qos: ParameterList,
    receiving_datareader_crypto: DatareaderCryptoHandle,
    sending_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<SerializedPayload> {
    todo!();
  }
}
