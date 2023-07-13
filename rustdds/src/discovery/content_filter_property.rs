use speedy::{Context, Readable, Reader, Writable, Writer};

use crate::serialization::speedy_pl_cdr_helpers::*;

/// The ContentFilterProperty field provides all the required information to
/// enable content filtering on the Writer side. For example, for the default
/// DDSSQL filter class, a valid filter expression for a data type containing
/// members a, b and c could be “(a < 5) AND (b == %0) AND (c >= %1)” with
/// expression parameters “5” and “3.” In order for the Writer to apply
/// the filter, it must have been configured to handle filters of the specified
/// filter class. If not, the Writer will simply ignore the filter information
/// and not filter any data samples.

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ContentFilterProperty {
  /// Name of the Content-filtered Topic associated with the Reader.
  /// Must have non-zero length.
  pub content_filtered_topic_name: String,

  /// Name of the Topic related to the Content-filtered Topic.
  /// Must have non-zero length.
  pub related_topic_name: String,

  /// Identifies the filter class this filter belongs to. RTPS can support
  /// multiple filter classes (SQL, regular expressions, custom filters,
  /// etc). Must have non-zero length.
  /// RTPS predefines the following values:
  /// “DDSSQL” Default filter class name if none specified.
  /// Matches the SQL filter specified by DDS, which must be available in all
  /// implementations.
  pub filter_class_name: String,

  /// The actual filter expression. Must be a valid expression for the filter
  /// class specified using filter_class_name.
  /// Must have non-zero length.
  pub filter_expression: String,

  /// Defines the value for each parameter in the filter expression.
  /// Can have zero length if the filter expression contains no parameters.
  pub expression_parameters: Vec<String>,
}

// TODO: This is patchy hack.
// Speedy reader/writer implementations do not respect
// alignment. A string starts with 4-byte character count, so
// we align to 4 bytes BEFORE each string reading operation, by the
// misalignment caused previously.
// Note: we must align BEFORE string read, not after.
// If string were followed by e.g. "u8", that would not have alignment applied.
impl<'a, C: Context> Readable<'a, C> for ContentFilterProperty {
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let cftn: StringWithNul = reader.read_value()?;

    read_pad(reader, cftn.len(), 4)?; // pad according to previous read
    let rtn: StringWithNul = reader.read_value()?;

    read_pad(reader, rtn.len(), 4)?;
    let fcn: StringWithNul = reader.read_value()?;

    read_pad(reader, fcn.len(), 4)?;
    let fe: StringWithNul = reader.read_value()?;

    let mut eps: Vec<String> = Vec::with_capacity(2);

    read_pad(reader, fe.len(), 4)?;
    let count = reader.read_u32()?;

    let mut prev_len = 0;
    for _ in 0..count {
      read_pad(reader, prev_len, 4)?;
      let s: StringWithNul = reader.read_value()?;
      prev_len = s.len();
      eps.push(s.into());
    }

    Ok(ContentFilterProperty {
      content_filtered_topic_name: cftn.into(),
      related_topic_name: rtn.into(),
      filter_class_name: fcn.into(),
      filter_expression: fe.into(),
      expression_parameters: eps,
    })
  }
}

// Writing several strings is a bit complicated, because
// we have to keep track of alignment.
// Again, alignment comes BEFORE string length, or vector item count, not after
// string.
impl<C: Context> Writable<C> for ContentFilterProperty {
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    let s1 = StringWithNul::from(self.content_filtered_topic_name.clone());
    // nothing yet to pad
    writer.write_value(&s1)?;

    let s2 = StringWithNul::from(self.related_topic_name.clone());
    write_pad(writer, s1.len(), 4)?;
    writer.write_value(&s2)?;

    let s3 = StringWithNul::from(self.filter_class_name.clone());
    write_pad(writer, s2.len(), 4)?;
    writer.write_value(&s3)?;

    let s4 = StringWithNul::from(self.filter_expression.clone());
    write_pad(writer, s3.len(), 4)?;
    writer.write_value(&s4)?;

    // write vector length
    write_pad(writer, s4.len(), 4)?;
    writer.write_u32(self.expression_parameters.len() as u32)?;

    let mut prev_len = 0;
    for ep in self.expression_parameters.iter().cloned() {
      write_pad(writer, prev_len, 4)?;
      let sn = StringWithNul::from(ep);
      writer.write_value(&sn)?;
      prev_len = sn.len();
    }

    Ok(())
  }
}
