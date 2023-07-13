use enumflags2::BitFlags;

use crate::dds::sampleinfo::*;

/// This is used to specify which samples are to be read or taken from
/// a [`Datareader`](crate::with_key::DataReader)
///
/// To be selected, the current state of the sample must be included in the
/// corresponding bitflags.
/// See DDS Specification 1.4 Section "2.2.2.5.8 ReadCondition"
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ReadCondition {
  sample_state_mask: BitFlags<SampleState>,
  view_state_mask: BitFlags<ViewState>,
  instance_state_mask: BitFlags<InstanceState>,
  /* Extension idea: Add a query string and a list of query parameters to upgrade this
   * to QueryCondition. But that would be a lot of work, especially in DataReader. */
}

impl ReadCondition {
  /// Condition reads all available samples
  pub fn any() -> Self {
    Self {
      sample_state_mask: SampleState::any(),
      view_state_mask: ViewState::any(),
      instance_state_mask: InstanceState::any(),
    }
  }

  /// Condition reads samples that are not already read
  pub fn not_read() -> Self {
    Self {
      sample_state_mask: SampleState::NotRead.into(),
      view_state_mask: ViewState::any(),
      instance_state_mask: InstanceState::any(),
    }
  }

  pub fn sample_state_mask(&self) -> &BitFlags<SampleState> {
    &self.sample_state_mask
  }

  pub fn view_state_mask(&self) -> &BitFlags<ViewState> {
    &self.view_state_mask
  }

  pub fn instance_state_mask(&self) -> &BitFlags<InstanceState> {
    &self.instance_state_mask
  }
}
