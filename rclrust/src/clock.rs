use anyhow::{ensure, Result};

use crate::error::ToRclRustResult;
use crate::impl_from_trait_for_enum;
use crate::log::Logger;
use crate::rclrust_error;
use crate::time::Time;

/// Time source type, used to indicate the source of a time measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockType {
    /// Clock is uninitialized yet.
    Uninitialized,
    /// Clock of this type will report the latest value reported by a ROS time source, or
    /// if a ROS time source is not active it reports the same as RCL_SYSTEM_TIME.
    /// For more information about ROS time sources, refer to the design document:
    /// http://design.ros2.org/articles/clock_and_time.html
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
pub(crate) struct RclClock(rcl_sys::rcl_clock_t);

unsafe impl Send for RclClock {}

impl RclClock {
    pub fn new(clock_type: ClockType) -> Result<Self> {
        ensure!(
            clock_type != ClockType::Uninitialized,
            "`ClockType::Uninitialized` is invalid type."
        );

        let mut clock = rcl_sys::rcl_clock_t {
            type_: ClockType::Uninitialized.into(),
            jump_callbacks: std::ptr::null_mut(),
            num_jump_callbacks: 0,
            get_now: None,
            data: std::ptr::null_mut(),
            allocator: unsafe { rcl_sys::rcutils_get_default_allocator() },
        };

        let mut allocator = unsafe { rcl_sys::rcutils_get_default_allocator() };
        unsafe {
            rcl_sys::rcl_clock_init(clock_type.into(), &mut clock, &mut allocator).to_result()?;
        }
        Ok(Self(clock))
    }

    pub fn now(&mut self) -> Result<Time> {
        let mut nanosecs = 0;

        unsafe {
            rcl_sys::rcl_clock_get_now(&mut self.0, &mut nanosecs).to_result()?;
        }
        Ok(Time::from_nanosecs(nanosecs, self.clock_type()))
    }

    pub fn clock_type(&self) -> ClockType {
        self.0.type_.into()
    }

    pub fn valid(&mut self) -> bool {
        unsafe { rcl_sys::rcl_clock_valid(&mut self.0) }
    }
}

impl Drop for RclClock {
    fn drop(&mut self) {
        if self.clock_type() != ClockType::Uninitialized {
            let ret = unsafe { rcl_sys::rcl_clock_fini(&mut self.0).to_result() };
            if let Err(e) = ret {
                rclrust_error!(
                    Logger::new("rclrust"),
                    "Failed to clean up rcl clock handle: {}",
                    e
                )
            }
        }
    }
}

#[derive(Debug)]
pub struct Clock(RclClock);

impl Clock {
    pub(crate) fn new(clock_type: ClockType) -> Result<Self> {
        let handle = RclClock::new(clock_type)?;
        Ok(Self(handle))
    }

    /// Construct a new `Clock`
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

    /// Construct a new `Clock`
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

    /// Construct a new `Clock`
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
        self.0.now()
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
        self.0.clock_type()
    }

    /// # Examples
    ///
    /// ```
    /// use rclrust::{Clock, ClockType};
    ///
    /// let mut clock = Clock::ros().unwrap();
    /// assert!(clock.valid());
    /// ```
    pub fn valid(&mut self) -> bool {
        self.0.valid()
    }
}
