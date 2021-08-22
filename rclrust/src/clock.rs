use std::mem::MaybeUninit;

use anyhow::{ensure, Context, Result};

use crate::{
    error::ToRclRustResult, impl_from_trait_for_enum, log::Logger, rclrust_error, time::Time,
};

/// Time source type, used to indicate the source of a time measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockType {
    /// Clock is uninitialized yet.
    Uninitialized,
    /// Clock of this type will report the latest value reported by a ROS time source, or
    /// if a ROS time source is not active it reports the same as RCL_SYSTEM_TIME.
    /// For more information about ROS time sources, refer to the design document:
    /// <http://design.ros2.org/articles/clock_and_time.html>
    RosTime,
    /// Clock of this type reports the same value as the system clock.
    SystemTime,
    /// Clock of this type reports a value from a monotonically increasing clock.
    SteadyTime,
}

impl_from_trait_for_enum! {
    ClockType,
    rcl_sys::rcl_clock_type_t,
    Uninitialized := RCL_CLOCK_UNINITIALIZED,
    RosTime := RCL_ROS_TIME,
    SystemTime := RCL_SYSTEM_TIME,
    SteadyTime := RCL_STEADY_TIME,
}

#[derive(Debug)]
pub struct Clock(rcl_sys::rcl_clock_t);

unsafe impl Send for Clock {}

impl Clock {
    pub(crate) fn new(clock_type: ClockType) -> Result<Self> {
        ensure!(
            clock_type != ClockType::Uninitialized,
            "`ClockType::Uninitialized` is invalid type."
        );

        let mut clock = MaybeUninit::uninit();
        unsafe {
            rcl_sys::rcl_clock_init(
                clock_type.into(),
                clock.as_mut_ptr(),
                &mut rcl_sys::rcutils_get_default_allocator(),
            )
            .to_result()
            .with_context(|| "rcl_sys::rcl_clock_init in Clock::new")?;
            Ok(Self(clock.assume_init()))
        }
    }

    pub(crate) fn raw_mut(&mut self) -> &mut rcl_sys::rcl_clock_t {
        &mut self.0
    }

    /// Construct a new `Clock` with ros time
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::{Clock, ClockType};
    ///
    /// let clock = Clock::ros().unwrap();
    /// assert_eq!(clock.clock_type(), ClockType::RosTime);
    /// ```
    pub fn ros() -> Result<Self> {
        Self::new(ClockType::RosTime)
    }

    /// Construct a new `Clock` with system time
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::{Clock, ClockType};
    ///
    /// let clock = Clock::system().unwrap();
    /// assert_eq!(clock.clock_type(), ClockType::SystemTime);
    /// ```
    pub fn system() -> Result<Self> {
        Self::new(ClockType::SystemTime)
    }

    /// Construct a new `Clock` with steady time
    ///
    /// # Examples
    ///
    /// ```
    /// use rclrust::{Clock, ClockType};
    ///
    /// let clock = Clock::steady().unwrap();
    /// assert_eq!(clock.clock_type(), ClockType::SteadyTime);
    /// ```
    pub fn steady() -> Result<Self> {
        Self::new(ClockType::SteadyTime)
    }

    /// # Examples
    ///
    /// ```
    /// use rclrust::{Clock, ClockType};
    ///
    /// let mut clock = Clock::ros().unwrap();
    /// let now = clock.now().unwrap();
    /// println!("current time: {:?}", now);
    /// ```
    pub fn now(&mut self) -> Result<Time> {
        let mut nanosecs = 0;

        unsafe {
            rcl_sys::rcl_clock_get_now(&mut self.0, &mut nanosecs)
                .to_result()
                .with_context(|| "rcl_sys::rcl_clock_get_now in Clock::now")?;
        }
        Ok(Time::from_nanosecs(nanosecs, self.clock_type()))
    }

    /// # Examples
    ///
    /// ```
    /// use rclrust::{Clock, ClockType};
    ///
    /// let clock = Clock::ros().unwrap();
    /// assert_eq!(clock.clock_type(), ClockType::RosTime);
    /// ```
    pub fn clock_type(&self) -> ClockType {
        self.0.type_.into()
    }

    /// # Examples
    ///
    /// ```
    /// use rclrust::{Clock, ClockType};
    ///
    /// let mut clock = Clock::ros().unwrap();
    /// assert!(clock.is_valid());
    /// ```
    pub fn is_valid(&mut self) -> bool {
        unsafe { rcl_sys::rcl_clock_valid(&mut self.0) }
    }
}

impl Drop for Clock {
    fn drop(&mut self) {
        if self.clock_type() != ClockType::Uninitialized {
            if let Err(e) = unsafe { rcl_sys::rcl_clock_fini(&mut self.0).to_result() } {
                rclrust_error!(
                    Logger::new("rclrust"),
                    "Failed to clean up rcl clock handle: {}",
                    e
                )
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn ros_clock_now() -> Result<()> {
        let mut clock = Clock::ros()?;
        let _now = clock.now()?;
        Ok(())
    }

    #[test]
    fn system_clock_now() -> Result<()> {
        let mut clock = Clock::system()?;
        let _now = clock.now()?;

        Ok(())
    }

    #[test]
    fn steady_clock_now() -> Result<()> {
        let mut clock = Clock::steady()?;
        let _now = clock.now()?;
        Ok(())
    }
}
