use std::{
  cmp::min,
  fmt::Debug,
  hash::Hash,
  mem::size_of,
  ops::{Bound, RangeBounds},
};
//use crate::messages::fragment_number::FragmentNumber;
use std::collections::BTreeSet;

use num_traits::NumOps;
use num_derive::{FromPrimitive, NumOps, ToPrimitive};
use serde::{Deserialize, Serialize};
use speedy::{Context, Readable, Reader, Writable, Writer};
use log::error;

//
/// RTPS Specification v2.3 Section "8.3.5.4 SequenceNumber"
#[derive(
  Copy,
  Clone,
  Debug,
  Hash,
  PartialEq,
  Eq,
  PartialOrd,
  Ord,
  NumOps,
  FromPrimitive,
  ToPrimitive,
  Serialize,
  Deserialize,
)]
pub struct SequenceNumber(i64);

impl SequenceNumber {
  pub const SEQUENCENUMBER_UNKNOWN: Self = Self((std::u32::MAX as i64) << 32);

  pub fn new(value: i64) -> Self {
    Self::from(value)
  }
  // Zero interface may be still in unstable Rust
  pub const fn zero() -> Self {
    Self(0)
  }
}

impl SequenceNumber {
  pub fn range_inclusive(begin: Self, end: Self) -> SequenceNumberRange {
    SequenceNumberRange::new(begin, end)
  }
}

impl From<i64> for SequenceNumber {
  fn from(value: i64) -> Self {
    Self(value)
  }
}

impl From<i32> for SequenceNumber {
  fn from(value: i32) -> Self {
    Self(value.into())
  }
}

impl From<usize> for SequenceNumber {
  fn from(value: usize) -> Self {
    Self(value as i64)
  }
}

impl From<SequenceNumber> for i64 {
  fn from(sequence_number: SequenceNumber) -> Self {
    sequence_number.0
  }
}

#[derive(Clone, Copy, Debug)]
pub struct SequenceNumberRange {
  begin: SequenceNumber,
  end: SequenceNumber,
}

impl SequenceNumberRange {
  pub fn new(begin: SequenceNumber, end: SequenceNumber) -> Self {
    Self { begin, end }
  }

  pub fn begin(&self) -> SequenceNumber {
    self.begin
  }

  pub fn end(&self) -> SequenceNumber {
    self.end
  }
}

impl Iterator for SequenceNumberRange {
  type Item = SequenceNumber;
  fn next(&mut self) -> Option<Self::Item> {
    if self.begin > self.end {
      None
    } else {
      let b = self.begin;
      self.begin = b + SequenceNumber::new(1);
      Some(b)
    }
  }
}

impl RangeBounds<SequenceNumber> for SequenceNumberRange {
  fn start_bound(&self) -> Bound<&SequenceNumber> {
    Bound::Included(&self.begin)
  }
  fn end_bound(&self) -> Bound<&SequenceNumber> {
    Bound::Included(&self.end)
  }
}

mod sequence_number_checked {
  use super::SequenceNumber;
  checked_impl!(CheckedAdd, checked_add, SequenceNumber);
  checked_impl!(CheckedSub, checked_sub, SequenceNumber);
  checked_impl!(CheckedMul, checked_mul, SequenceNumber);
  checked_impl!(CheckedDiv, checked_div, SequenceNumber);
}
// SequenceNumber serialization:
//
// RTPS Spec v2.3 Section 9.4.2.5:
// SequenceNumber is serialized 32 bit high word first, then low 32 bits,
// regardless of endianness. Then within those 32 bit words, ecncoding
// endianness is followed.
// E.g. SequenceNumber(1) is encoded in 8 little-endian bytes as:
// 00 00 00 00 01 00 00 00
impl<'a, C: Context> Readable<'a, C> for SequenceNumber {
  #[inline]
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let high: i32 = reader.read_value()?;
    let low: u32 = reader.read_value()?;

    Ok(Self(((i64::from(high)) << 32) + i64::from(low)))
  }

  #[inline]
  fn minimum_bytes_needed() -> usize {
    size_of::<Self>()
  }
}

impl<C: Context> Writable<C> for SequenceNumber {
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    writer.write_i32((self.0 >> 32) as i32)?;
    writer.write_u32(self.0 as u32)?;
    Ok(())
  }
}

impl Default for SequenceNumber {
  fn default() -> Self {
    Self(1)
  }
}

// ---------------------------------------------------------------
// ---------------------------------------------------------------

#[derive(
  Copy,
  Clone,
  Debug,
  Hash,
  PartialOrd,
  PartialEq,
  Ord,
  Eq,
  Readable,
  Writable,
  NumOps,
  FromPrimitive,
  ToPrimitive,
)]
pub struct FragmentNumber(u32);

impl FragmentNumber {
  pub const INVALID: Self = Self(0); // Valid FragmentNumbers start at 1.

  pub fn new(value: u32) -> Self {
    FragmentNumber(value)
  }
  pub fn range_inclusive(begin: Self, end: Self) -> FragmentNumberRange {
    FragmentNumberRange::new(begin, end)
  }
}

impl Default for FragmentNumber {
  fn default() -> Self {
    Self(1)
  }
}

impl From<u32> for FragmentNumber {
  fn from(value: u32) -> Self {
    Self(value)
  }
}

impl From<FragmentNumber> for u32 {
  fn from(fragment_number: FragmentNumber) -> Self {
    fragment_number.0
  }
}

impl From<FragmentNumber> for usize {
  fn from(fragment_number: FragmentNumber) -> Self {
    fragment_number.0 as usize
  }
}

// to make this fit into NumberSet<N>
impl From<i64> for FragmentNumber {
  fn from(value: i64) -> Self {
    Self(value as u32)
  }
}

// to make this fit into NumberSet<N>
impl From<FragmentNumber> for i64 {
  fn from(fragment_number: FragmentNumber) -> Self {
    fragment_number.0.into()
  }
}

#[derive(Clone, Copy, Debug)]
pub struct FragmentNumberRange {
  begin: FragmentNumber,
  end: FragmentNumber,
}

impl FragmentNumberRange {
  pub fn new(begin: FragmentNumber, end: FragmentNumber) -> Self {
    Self { begin, end }
  }

  pub fn begin(&self) -> FragmentNumber {
    self.begin
  }

  pub fn end(&self) -> FragmentNumber {
    self.end
  }
}

impl Iterator for FragmentNumberRange {
  type Item = FragmentNumber;
  fn next(&mut self) -> Option<Self::Item> {
    if self.begin > self.end {
      None
    } else {
      let b = self.begin;
      self.begin = b + FragmentNumber::new(1);
      Some(b)
    }
  }
}

impl RangeBounds<FragmentNumber> for FragmentNumberRange {
  fn start_bound(&self) -> Bound<&FragmentNumber> {
    Bound::Included(&self.begin)
  }
  fn end_bound(&self) -> Bound<&FragmentNumber> {
    Bound::Included(&self.end)
  }
}

mod fragment_number_checked {
  use super::FragmentNumber;
  checked_impl!(CheckedAdd, checked_add, FragmentNumber);
  checked_impl!(CheckedSub, checked_sub, FragmentNumber);
  checked_impl!(CheckedMul, checked_mul, FragmentNumber);
  checked_impl!(CheckedDiv, checked_div, FragmentNumber);
}
// ---------------------------------------------------------------

pub type SequenceNumberSet = NumberSet<SequenceNumber>;
pub type FragmentNumberSet = NumberSet<FragmentNumber>;

// ---------------------------------------------------------------

#[derive(Clone, Debug, Hash, PartialEq, Eq)]
pub struct NumberSet<N>
where
  N: Clone + Debug + Hash + PartialEq + Eq + NumOps + From<i64>,
{
  bitmap_base: N,
  num_bits: u32,
  bitmap: Vec<u32>, /* .len() == (numBits+31)/32
                     * bitmap bits are numbered from MSB to LSB. Bit 0 (MSB of bitmap[0])
                     * represents SequenceNumber bitmap_base. Bit 31 (LSB of bitmap[0])
                     * represents SequenceNumber (bitmap_base + 31).
                     * When num_bits == 0 , bitmap.len() == 0 */
}

// Not that empty sets also have a valid "base".

impl<N> NumberSet<N>
where
  N: Clone + Copy + Debug + Hash + PartialEq + Eq + NumOps + From<i64> + Ord + PartialOrd,
  i64: From<N>,
{
  // Construct an empy set from given base number
  pub fn new(bitmap_base: N, num_bits: u32) -> Self {
    let word_count = (num_bits + 31) / 32;
    Self {
      bitmap_base,
      num_bits,
      bitmap: vec![0; word_count as usize],
    }
  }

  pub fn base(&self) -> N {
    self.bitmap_base
  }

  pub fn new_empty(bitmap_base: N) -> Self {
    Self::new(bitmap_base, 0)
  }

  #[allow(dead_code)] // Is this really unneccessary?
  pub fn is_empty(&self) -> bool {
    self.num_bits == 0 || self.iter().next().is_none()
  }

  #[cfg(test)]
  pub fn test_insert(&mut self, sn: N) {
    self.insert(sn);
  }

  fn insert(&mut self, sn: N) {
    if sn < self.bitmap_base
      || self.num_bits == 0
      || sn >= self.bitmap_base + N::from(self.num_bits as i64)
    {
      error!("out of bounds .insert({:?}) to {:?}", sn, self);
    } else {
      let bit_pos = i64::from(sn - self.bitmap_base) as u32;
      let word_num = bit_pos / 32;
      let bit_num = bit_pos % 32;
      if word_num >= self.bitmap.len() as u32 {
        error!(
          "Tried to index {:?} (bit_pos={:?}) (word {:?} bit {:?}) in {:?}",
          sn, bit_pos, word_num, bit_num, self
        );
      }
      self.bitmap[word_num as usize] |= 1u32 << (31 - bit_num);
    }
  }

  /// Construct a new Numberset from base and set
  /// base is the index of the first element of the bitmap
  /// set is the set if Numbers that will be contained in the set.
  /// Highest possible number in set is base+255.
  pub fn from_base_and_set(base: N, set: &BTreeSet<N>) -> Self {
    match (set.iter().next(), set.iter().next_back()) {
      (Some(&start), Some(&end)) => {
        // sanity
        let base = if start < base {
          error!(
            "from_base_and_set : need base <= set start: base={:?} and set {:?}",
            base, set
          );
          start
        } else {
          base
        };
        if base < N::from(1) {
          // RTPS v2.5 spec Section "8.3.5.5 SequenceNumberSet":
          // minimum(SequenceNumberSet) >= 1
          error!(
            "from_base_and_set : minimum possible set element is 1, got base={:?}",
            base
          );
          return Self::new_empty(N::from(1));
        }
        // start <= end, because BTreeSet properties.
        let end = if i64::from(end) - i64::from(base) >= 256 {
          // RTPS v2.5 spec Section "8.3.5.5 SequenceNumberSet":
          // maximum(SequenceNumberSet) - minimum(SequenceNumberSet) < 256
          let truncated_end = base + N::from(255);
          error!("from_base_and_set : max size (256) exceeded, base = {:?}, start = {:?} end = {:?}. Truncating end to {:?}",
              base, start, end, truncated_end );
          truncated_end
        } else {
          end
        };
        // sanity ok. Now do the actual work.
        //let num_bits = i64::from( end - base + N::from(1) );
        let mut sns = Self::new(base, i64::from(end) as u32);
        for s in set.iter().filter(|s| base <= **s && **s <= end) {
          sns.insert(*s);
        }
        sns
      }
      (_, _) => Self::new_empty(base),
    }
  }

  pub fn iter(&self) -> NumberSetIter<N> {
    NumberSetIter::<N> {
      seq: self,
      at_bit: 0,
      rev_at_bit: self.num_bits,
    }
  }

  pub fn len_serialized(&self) -> usize {
    size_of::<N>() // base
    + 4 // num_bits
    + 4 * ((self.num_bits as usize +31)/32) // bitmap
  }
}

impl<'a, C: Context, N> Readable<'a, C> for NumberSet<N>
where
  N:
    Clone + Debug + Hash + PartialEq + Eq + NumOps + From<i64> + Ord + PartialOrd + Readable<'a, C>,
  i64: From<N>,
{
  #[inline]
  fn read_from<R: Reader<'a, C>>(reader: &mut R) -> Result<Self, C::Error> {
    let bitmap_base: N = reader.read_value()?;
    let num_bits: u32 = reader.read_value()?;
    let word_count = (num_bits + 31) / 32;
    let mut bitmap: Vec<u32> = Vec::with_capacity(word_count as usize);
    for _ in 0..word_count {
      bitmap.push(reader.read_value()?);
    }
    Ok(Self {
      bitmap_base,
      num_bits,
      bitmap,
    })
  }

  #[inline]
  fn minimum_bytes_needed() -> usize {
    size_of::<N>() + size_of::<u32>()
  }
}

impl<C: Context, N> Writable<C> for NumberSet<N>
where
  N: Clone + Debug + Hash + PartialEq + Eq + NumOps + From<i64> + Ord + PartialOrd + Writable<C>,
{
  #[inline]
  fn write_to<T: ?Sized + Writer<C>>(&self, writer: &mut T) -> Result<(), C::Error> {
    writer.write_value(&self.bitmap_base)?;
    writer.write_u32(self.num_bits)?;
    let word_count = (self.num_bits + 31) / 32;
    let bitmap_len = self.bitmap.len() as u32;
    // sanity check
    if bitmap_len != word_count {
      error!(
        "SequenceNumberSet bitmap.len() = {} but word_count = {}",
        self.bitmap.len(),
        word_count
      );
    }
    //TODO: If the sanity check above fails, we may write the wrong number of
    // words. This is highly suspicious.
    for i in 0..min(word_count, bitmap_len) {
      writer.write_u32(self.bitmap[i as usize])?;
    }
    Ok(())
  }
}

#[derive(Clone, Debug, Hash, PartialEq, Eq)]
pub struct NumberSetIter<'a, N>
where
  N: Clone + Debug + Hash + PartialEq + Eq + NumOps + From<i64> + Ord + PartialOrd,
{
  seq: &'a NumberSet<N>,
  at_bit: u32,
  rev_at_bit: u32,
}

impl<N> Iterator for NumberSetIter<'_, N>
where
  N: Clone + Copy + Debug + Hash + PartialEq + Eq + NumOps + From<i64> + Ord + PartialOrd,
{
  type Item = N;

  fn next(&mut self) -> Option<Self::Item> {
    //TODO: This probably could made faster with the std function
    // .leading_zeroes() in type u32 to do several iterations of the loop in
    // one step, given that we have clz as a machine instruction or short sequence.
    while self.at_bit < self.rev_at_bit {
      // bit indexing formula from RTPS spec v2.3 Section 9.4.2.6
      let have_one =
        self.seq.bitmap[(self.at_bit / 32) as usize] & (1 << (31 - self.at_bit % 32)) != 0;
      self.at_bit += 1;
      if have_one {
        return Some(N::from(i64::from(self.at_bit - 1)) + self.seq.bitmap_base);
      }
    }
    None
  }
}

impl<N> DoubleEndedIterator for NumberSetIter<'_, N>
where
  N: Clone + Copy + Debug + Hash + PartialEq + Eq + NumOps + From<i64> + Ord + PartialOrd,
{
  fn next_back(&mut self) -> Option<Self::Item> {
    while self.at_bit < self.rev_at_bit {
      // bit indexing formula from RTPS spec v2.3 Section 9.4.2.6
      self.rev_at_bit -= 1;
      let have_one =
        self.seq.bitmap[(self.rev_at_bit / 32) as usize] & (1 << (31 - self.rev_at_bit % 32)) != 0;
      if have_one {
        return Some(N::from(i64::from(self.rev_at_bit)) + self.seq.bitmap_base);
      }
    }
    None
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn sequence_number_starts_by_default_from_one() {
    assert_eq!(SequenceNumber::from(1), SequenceNumber::default());
  }

  #[test]
  fn fragment_number_starts_by_default_from_one() {
    assert_eq!(FragmentNumber::from(1u32), FragmentNumber::default());
  }

  serialization_test!( type = FragmentNumber,
  {
      fragment_number_zero,
      FragmentNumber::from(0u32),
      le = [0x00, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x00]
  },
  {
      fragment_number_default,
      FragmentNumber::default(),
      le = [0x01, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x01]
  },
  {
      fragment_number_non_zero,
      FragmentNumber::from(0xDEADBEEFu32),
      le = [0xEF, 0xBE, 0xAD, 0xDE],
      be = [0xDE, 0xAD, 0xBE, 0xEF]
  });
  serialization_test!( type = SequenceNumber,
  {
      sequence_number_default,
      SequenceNumber::default(),
      le = [0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00],
      be = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]
  },
  {
      sequence_number_unknown,
      SequenceNumber::SEQUENCENUMBER_UNKNOWN,
      le = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00],
      be = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00]
  },
  {
      sequence_number_non_zero,
      SequenceNumber::from(0x0011223344556677i64),
      le = [0x33, 0x22, 0x11, 0x00, 0x77, 0x66, 0x55, 0x44],
      be = [0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]
  });

  serialization_test!( type = SequenceNumberSet,
  {
      sequence_number_set_empty,
      SequenceNumberSet::new_empty(SequenceNumber::from(42)),
      le = [0x00, 0x00, 0x00, 0x00,  // bitmapBase
            0x2A, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00], // numBits
      be = [0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x2A,
            0x00, 0x00, 0x00, 0x00]
  },
  {
    sequence_number_set_one,
    {
        let mut set = SequenceNumberSet::new(SequenceNumber::from(1),1);
        set.insert(SequenceNumber::from(1));
        set
      },
      le = [0x00, 0x00, 0x00, 0x00, // bitmapBase high
            0x01, 0x00, 0x00, 0x00, // bitmapBase low
            0x01, 0x00, 0x00, 0x00, // bitmap bit count
            0x00, 0x00, 0x00, 0x80], // bitmap word 0
      // The last word here has 11 bits set from MSB end, and then 25-11 = 14 zeroes.
      // The last 32-25 = 7 bits are undefined. Our implementation sets them to zero,
      // but others may set to ones.
      // 0xffc0_00YZ, where  YZ & 0x80 = 0x00, but otherwise undefined
      // So e.g. 0x7f is valid least siginifanct byte, as is 0x00, or 0x0f.
      be = [0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x01,
            0x80, 0x00, 0x00, 0x00]

  },
  {
      sequence_number_set_manual,
      {
        let mut set = SequenceNumberSet::new(SequenceNumber::from(1),25);
        for sn in 1..11 {
          set.insert(SequenceNumber::from(sn));
        }
        set
      },
      le = [0x00, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00,
            0x19, 0x00, 0x00, 0x00,
            0x00, 0x00, 0xc0, 0xff],
      // The last word here has 11 bits set from MSB end, and then 25-11 = 14 zeroes.
      // The last 32-25 = 7 bits are undefined. Our implementation sets them to zero,
      // but others may set to ones.
      // 0xffc0_00YZ, where  YZ & 0x80 = 0x00, but otherwise undefined
      // So e.g. 0x7f is valid least siginifanct byte, as is 0x00, or 0x0f.
      be = [0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01,
            0x00, 0x00, 0x00, 0x19,
            0xff, 0xc0, 0x00, 0x00]
  },
  {
      sequence_number_set_multiword,
      {
        // send sequence numbers 10,11,12,..,52. (43 ones) Set goes up to 64.
        let mut set = SequenceNumberSet::new(SequenceNumber::from(10),64);
        for sn in 10..=52 {
          set.insert(SequenceNumber::from(sn));
        }
        set
      },
      le = [0x00, 0x00, 0x00, 0x00,
            0x0A, 0x00, 0x00, 0x00, // base = 10
            0x40, 0x00, 0x00, 0x00, // 0x40 = 64 bits in map => 2 words
            0xff, 0xff, 0xff, 0xff, // bits 10..41 are all ones
            0x00, 0x00, 0xe0, 0xff, // bits 42..51 (10 pcs) are ones, rest zero
            ],
      be = [0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x0A,
            0x00, 0x00, 0x00, 0x40,
            0xff, 0xff, 0xff, 0xff,
            0xff, 0xe0, 0x00, 0x00]
  });

  serialization_test!( type = FragmentNumberSet,
  {
    fragment_number_set_empty,
    FragmentNumberSet::new_empty(FragmentNumber::from(42u32) ),
    le = [0x2A, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00],
    be = [0x00, 0x00, 0x00, 0x2A,
          0x00, 0x00, 0x00, 0x00]
  },
  {
      fragment_number_set_manual,
      {
          let mut set = FragmentNumberSet::new(FragmentNumber::from(1000u32), 14);
          set.insert(FragmentNumber::from(1001u32));
          set.insert(FragmentNumber::from(1003u32));
          set.insert(FragmentNumber::from(1004u32));
          set.insert(FragmentNumber::from(1006u32));
          set.insert(FragmentNumber::from(1008u32));
          set.insert(FragmentNumber::from(1010u32));
          set.insert(FragmentNumber::from(1013u32));
          set
      },
      le = [0xE8, 0x03, 0x00, 0x00,
            0x0E, 0x00, 0x00, 0x00,
            0x00, 0x00, 0xA4, 0x5A],
      be = [0x00, 0x00, 0x03, 0xE8,
            0x00, 0x00, 0x00, 0x0E,
            0x5A, 0xA4, 0x00, 0x00]
  });
}
