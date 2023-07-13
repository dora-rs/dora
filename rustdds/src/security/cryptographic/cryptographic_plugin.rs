use crate::{
  messages::submessages::elements::{
    crypto_content::CryptoContent, parameter_list::ParameterList,
    serialized_payload::SerializedPayload,
  },
  rtps::{Message, Submessage},
  security::{
    access_control::types::*, authentication::types::*, cryptographic::types::*, types::*,
  },
};
// Imports for doc references
#[cfg(doc)]
use crate::{messages::submessages::submessage::SecuritySubmessage, rtps::SubmessageBody};

/// CryptoKeyFactory: section 8.5.1.7 of the Security specification (v. 1.1)
pub trait CryptoKeyFactory {
  /// register_local_participant: section 8.5.1.7.1 of the Security
  /// specification (v. 1.1)
  fn register_local_participant(
    participant_identity: IdentityHandle,
    participant_permissions: PermissionsHandle,
    participant_properties: Vec<Property>,
    participant_security_attributes: ParticipantSecurityAttributes,
  ) -> SecurityResult<ParticipantCryptoHandle>;

  /// register_matched_remote_participant: section 8.5.1.7.2 of the Security
  /// specification (v. 1.1)
  fn register_matched_remote_participant(
    local_participant_crypto_handle: ParticipantCryptoHandle,
    remote_participant_identity: IdentityHandle,
    remote_participant_permissions: PermissionsHandle,
    shared_secret: SharedSecretHandle,
  ) -> SecurityResult<ParticipantCryptoHandle>;

  /// register_local_datawriter: section 8.5.1.7.3 of the Security specification
  /// (v. 1.1)
  fn register_local_datawriter(
    participant_crypto: ParticipantCryptoHandle,
    datawriter_properties: Vec<Property>,
    datawriter_security_attributes: EndpointSecurityAttributes,
  ) -> SecurityResult<DatawriterCryptoHandle>;

  /// register_matched_remote_datareader: section 8.5.1.7.4 of the Security
  /// specification (v. 1.1)
  fn register_matched_remote_datareader(
    local_datawriter_crypto_handle: DatawriterCryptoHandle,
    remote_participant_crypto: ParticipantCryptoHandle,
    shared_secret: SharedSecretHandle,
    relay_only: bool,
  ) -> SecurityResult<DatareaderCryptoHandle>;

  /// register_local_datareader: section 8.5.1.7.5 of the Security specification
  /// (v. 1.1)
  fn register_local_datareader(
    participant_crypto: ParticipantCryptoHandle,
    datareader_properties: Vec<Property>,
    datareader_security_attributes: EndpointSecurityAttributes,
  ) -> SecurityResult<DatareaderCryptoHandle>;

  /// register_matched_remote_datawriter: section 8.5.1.7.6 of the Security
  /// specification (v. 1.1)
  fn register_matched_remote_datawriter(
    local_datareader_crypto_handle: DatareaderCryptoHandle,
    remote_participant_crypt: ParticipantCryptoHandle,
    shared_secret: SharedSecretHandle,
  ) -> SecurityResult<DatareaderCryptoHandle>;

  /// unregister_participant: section 8.5.1.7.7 of the Security specification
  /// (v. 1.1)
  fn unregister_participant(
    participant_crypto_handle: ParticipantCryptoHandle,
  ) -> SecurityResult<()>;
  /// unregister_datawriter: section 8.5.1.7.8 of the Security specification (v.
  /// 1.1)
  fn unregister_datawriter(datawriter_crypto_handle: DatawriterCryptoHandle) -> SecurityResult<()>;
  /// unregister_datareader: section 8.5.1.7.9 of the Security specification (v.
  /// 1.1)
  fn unregister_datareader(datareader_crypto_handle: DatareaderCryptoHandle) -> SecurityResult<()>;
}

/// CryptoKeyExchange: section 8.5.1.8 of the Security specification (v. 1.1)
pub trait CryptoKeyExchange {
  /// create_local_participant_crypto_tokens: section 8.5.1.8.1 of the Security
  /// specification (v. 1.1)
  ///
  /// In a vector, return the tokens that would be written in
  /// `local_participant_crypto_tokens`.
  fn create_local_participant_crypto_tokens(
    local_participant_crypto: ParticipantCryptoHandle,
    remote_participant_crypto: ParticipantCryptoHandle,
  ) -> SecurityResult<Vec<ParticipantCryptoToken>>;

  /// set_remote_participant_crypto_tokens: section 8.5.1.8.2 of the Security
  /// specification (v. 1.1)
  fn set_remote_participant_crypto_tokens(
    local_participant_crypto: ParticipantCryptoHandle,
    remote_participant_crypto: ParticipantCryptoHandle,
    remote_participant_tokens: Vec<ParticipantCryptoToken>,
  ) -> SecurityResult<()>;

  /// create_local_datawriter_crypto_tokens: section 8.5.1.8.3 of the Security
  /// specification (v. 1.1)
  ///
  /// In a vector, return the tokens that would be written in
  /// `local_datawriter_crypto_tokens`.
  fn create_local_datawriter_crypto_tokens(
    local_datawriter_crypto: DatawriterCryptoHandle,
    remote_datareader_crypto: DatareaderCryptoHandle,
  ) -> SecurityResult<Vec<DatawriterCryptoToken>>;

  /// set_remote_datawriter_crypto_tokens: section 8.5.1.8.4 of the Security
  /// specification (v. 1.1)
  fn set_remote_datawriter_crypto_tokens(
    local_datareader_crypto: DatareaderCryptoHandle,
    remote_datawriter_crypto: DatawriterCryptoHandle,
    remote_datawriter_tokens: Vec<DatawriterCryptoToken>,
  ) -> SecurityResult<()>;

  /// create_local_datareader_crypto_tokens: section 8.5.1.8.5 of the Security
  /// specification (v. 1.1)
  ///
  /// In a vector, return the tokens that would be written in
  /// `local_datareader_crypto_tokens`.
  fn create_local_datareader_crypto_tokens(
    local_datareader_crypto: DatareaderCryptoHandle,
    remote_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<Vec<DatareaderCryptoToken>>;

  /// set_remote_datareader_crypto_tokens: section 8.5.1.8.6 of the Security
  /// specification (v. 1.1)
  fn set_remote_datareader_crypto_tokens(
    local_datawriter_crypto: DatawriterCryptoHandle,
    remote_datareader_crypto: DatareaderCryptoHandle,
    remote_datareader_tokens: Vec<DatareaderCryptoToken>,
  ) -> SecurityResult<()>;

  /// return_crypto_tokens: section 8.5.1.8.7 of the Security specification (v.
  /// 1.1)
  fn return_crypto_tokens(crypto_tokens: Vec<CryptoToken>) -> SecurityResult<()>;
}

/// CryptoTransform: section 8.5.1.9 of the Security specification (v. 1.1)
///
/// Differs from the specification by returning the results instead of writing
/// them to provided buffers.
pub trait CryptoTransform {
  /// encode_serialized_payload: section 8.5.1.9.1 of the Security specification
  /// (v. 1.1)
  ///
  /// In a tuple, return the results that would be written in `encoded_buffer`
  /// and `extra_inline_qos`.
  fn encode_serialized_payload(
    plain_buffer: SerializedPayload,
    sending_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<(CryptoContent, ParameterList)>;

  /// encode_datawriter_submessage: section 8.5.1.9.2 of the Security
  /// specification (v. 1.1)
  ///
  /// In an [EncodeResult], return the submessages that would be written in
  /// `encoded_rtps_submessage`.
  /// In case of [EncodeResult::One], the same result will be use for all
  /// receivers, while [EncodeResult::Many] shall contain a result for each
  /// receiving datareader in `receiving_datareader_crypto_list`.
  /// `receiving_datareader_crypto_list_index` is dropped.
  ///
  /// # Panics
  /// The function will panic if `plain_rtps_submessage.body` is not
  /// [SubmessageBody::Writer].
  fn encode_datawriter_submessage(
    plain_rtps_submessage: Submessage,
    sending_datawriter_crypto: DatawriterCryptoHandle,
    receiving_datareader_crypto_list: Vec<DatareaderCryptoHandle>,
  ) -> SecurityResult<EncodeResult<EncodedSubmessage>>;

  /// encode_datareader_submessage: section 8.5.1.9.3 of the Security
  /// specification (v. 1.1)
  ///
  /// In an [EncodeResult], return the submessages that would be written in
  /// `encoded_rtps_submessage`.
  /// In case of [EncodeResult::One], the same result will be use for all
  /// receivers, while [EncodeResult::Many] shall contain a result for each
  /// receiving datawriter in `receiving_datawriter_crypto_list`.
  ///
  /// # Panics
  /// The function will panic if `plain_rtps_submessage.body` is not
  /// [SubmessageBody::Reader].
  fn encode_datareader_submessage(
    plain_rtps_submessage: Submessage,
    sending_datareader_crypto: DatareaderCryptoHandle,
    receiving_datawriter_crypto_list: Vec<DatawriterCryptoHandle>,
  ) -> SecurityResult<EncodeResult<EncodedSubmessage>>;

  /// encode_rtps_message: section 8.5.1.9.4 of the Security specification (v.
  /// 1.1)
  ///
  /// In an [EncodeResult], return the message that would be written in
  /// `encoded_rtps_message`.
  /// In case of [EncodeResult::One], the same result will be use for all
  /// receivers, while [EncodeResult::Many] shall contain a result for each
  /// receiving participant in `receiving_participant_crypto_list`.
  /// in the case that no transformation is performed and the plain message
  /// should be sent, the plain message shall be returned in
  /// [EncodeResult::One] (instead of returning false, see the spec).
  /// `receiving_participant_crypto_list_index` is dropped.
  fn encode_rtps_message(
    plain_rtps_message: Message,
    sending_participant_crypto: ParticipantCryptoHandle,
    receiving_participant_crypto_list: Vec<ParticipantCryptoHandle>,
  ) -> SecurityResult<EncodeResult<Message>>;

  /// decode_rtps_message: section 8.5.1.9.5 of the Security specification (v.
  /// 1.1)
  ///
  /// Return the message that would be written in `plain_buffer`.
  fn decode_rtps_message(
    encoded_buffer: Message,
    receiving_participant_crypto: ParticipantCryptoHandle,
    sending_participant_crypto: ParticipantCryptoHandle,
  ) -> SecurityResult<Message>;

  /// preprocess_secure_submsg: section 8.5.1.9.6 of the Security specification
  /// (v. 1.1)
  ///
  /// Return the secure submessage category that would be written in
  /// `secure_submessage_category`. The [DatawriterCryptoHandle] and
  /// [DatareaderCryptoHandle] that would be written in `datawriter_crypto` and
  /// `datareader_crypto` shall be returned in
  /// [SecureSubmessageCategory::DatawriterSubmessage] or
  /// [SecureSubmessageCategory::DatareaderSubmessage] in the order
  /// (sender,receiver). In the case `INFO_SUBMESSAGE`,
  /// [SecureSubmessageCategory::InfoSubmessage] is returned instead of `false`.
  ///
  /// # Panics
  /// The function will panic if `encoded_rtps_submessage.body` is not
  /// [SubmessageBody::Security] wrapping [SecuritySubmessage::SecurePrefix].
  fn preprocess_secure_submsg(
    encoded_rtps_submessage: Submessage,
    receiving_participant_crypto: ParticipantCryptoHandle,
    sending_participant_crypto: ParticipantCryptoHandle,
  ) -> SecurityResult<SecureSubmessageCategory>;

  /// decode_datawriter_submessage: section 8.5.1.9.7 of the Security
  /// specification (v. 1.1)
  ///
  /// Return the writer submessage that would be written in
  /// `plain_rtps_submessage`.
  ///
  /// # Panics
  /// The function will panic if `encoded_rtps_submessage.0.body` and
  /// `encoded_rtps_submessage.2.body`  are not [SubmessageBody::Security]
  /// wrapping [SecuritySubmessage::SecurePrefix] and
  /// [SecuritySubmessage::SecurePostfix] respectively.
  fn decode_datawriter_submessage(
    encoded_rtps_submessage: (Submessage, Submessage, Submessage),
    receiving_datareader_crypto: DatareaderCryptoHandle,
    sending_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<Submessage>;

  /// decode_datareader_submessage: section 8.5.1.9.8 of the Security
  /// specification (v. 1.1)
  ///
  /// Return the reader submessage that would be written in
  /// `plain_rtps_submessage`.
  ///
  /// # Panics
  /// The function will panic if `encoded_rtps_submessage.0.body` and
  /// `encoded_rtps_submessage.2.body`  are not [SubmessageBody::Security]
  /// wrapping [SecuritySubmessage::SecurePrefix] and
  /// [SecuritySubmessage::SecurePostfix] respectively.
  fn decode_datareader_submessage(
    encoded_rtps_submessage: (Submessage, Submessage, Submessage),
    receiving_datawriter_crypto: DatawriterCryptoHandle,
    sending_datareader_crypto: DatareaderCryptoHandle,
  ) -> SecurityResult<Submessage>;

  /// decode_serialized_payload: section 8.5.1.9.9 of the Security specification
  /// (v. 1.1)
  ///
  /// Return the serialized payload that would be written in `plain_buffer`
  fn decode_serialized_payload(
    encoded_buffer: CryptoContent,
    inline_qos: ParameterList,
    receiving_datareader_crypto: DatareaderCryptoHandle,
    sending_datawriter_crypto: DatawriterCryptoHandle,
  ) -> SecurityResult<SerializedPayload>;
}
