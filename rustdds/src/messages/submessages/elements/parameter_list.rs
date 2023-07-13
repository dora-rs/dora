use std::collections::BTreeMap;

use bytes::Bytes;
use speedy::{Context, Readable, Writable, Writer};

use crate::{
  messages::submessages::elements::parameter::Parameter, structure::parameter_id::ParameterId,
};

/// ParameterList is used as part of several messages to encapsulate
/// QoS parameters that may affect the interpretation of the message.
/// The encapsulation of the parameters follows a mechanism that allows
/// extensions to the QoS without breaking backwards compatibility.
#[derive(Debug, PartialEq, Eq, Clone, Default)]
pub struct ParameterList {
  pub parameters: Vec<Parameter>,
}

impl ParameterList {
  pub fn new() -> Self {
    Self::default()
  }

  pub fn is_empty(&self) -> bool {
    self.parameters.is_empty()
  }

  pub fn len_serialized(&self) -> usize {
    self
      .parameters
      .iter()
      .map(|p| p.len_serialized())
      .sum::<usize>()
      + SENTINEL.len_serialized()
  }

  pub fn push(&mut self, p: Parameter) {
    self.parameters.push(p);
  }

  pub fn serialize_to_bytes(&self, endianness: speedy::Endianness) -> Result<Bytes, speedy::Error> {
    let b = self.write_to_vec_with_ctx(endianness)?;
    Ok(Bytes::from(b))
  }

  pub fn to_map(&self) -> BTreeMap<ParameterId, Vec<&Parameter>> {
    self.parameters.iter().fold(BTreeMap::new(), |mut m, p| {
      m.entry(p.parameter_id).or_insert(Vec::new()).push(p);
      m
    })
  }
}

const SENTINEL: Parameter = Parameter {
  parameter_id: ParameterId::PID_SENTINEL,
  value: vec![],
};

impl<C: Context> Writable<C> for ParameterList {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    for param in &self.parameters {
      writer.write_value(param)?;
    }
    // finally, write end marker.
    writer.write_value(&SENTINEL)?;

    Ok(())
  }
}

impl<'a, C: Context> Readable<'a, C> for ParameterList {
  #[inline]
  fn read_from<R: speedy::Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let mut parameters = Self::default();

    // loop ends in failure to read something or catching sentinel
    loop {
      let parameter_id = ParameterId::read_from(reader)?;
      let length = u16::read_from(reader)?;

      if parameter_id == ParameterId::PID_SENTINEL {
        // This is parameter list end marker.
        // We do not read its Parameter contents ("value"),
        // because it is of size zero by definition.
        return Ok(parameters);
      }

      parameters.parameters.push(Parameter {
        parameter_id,
        value: reader.read_vec(length as usize)?,
      });
    }
  }
}
