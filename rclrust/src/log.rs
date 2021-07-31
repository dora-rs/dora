use std::convert::{TryFrom, TryInto};
use std::ffi::CString;
use std::os::raw::{c_char, c_int};

use anyhow::{Context, Result};
use once_cell::sync::Lazy;
use parking_lot::ReentrantMutex;

use crate::error::{RclRustError, ToRclRustResult};
use crate::impl_from_trait_for_enum;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LogSeverity {
    Unset,
    Debug,
    Info,
    Warn,
    Error,
    Fatal,
}

impl_from_trait_for_enum! {
    LogSeverity,
    rcl_sys::RCUTILS_LOG_SEVERITY,
    Unset := RCUTILS_LOG_SEVERITY_UNSET,
    Debug := RCUTILS_LOG_SEVERITY_DEBUG,
    Info := RCUTILS_LOG_SEVERITY_INFO,
    Warn := RCUTILS_LOG_SEVERITY_WARN,
    Error := RCUTILS_LOG_SEVERITY_ERROR,
    Fatal := RCUTILS_LOG_SEVERITY_FATAL,
}

impl TryFrom<c_int> for LogSeverity {
    type Error = anyhow::Error;

    fn try_from(from: c_int) -> Result<Self> {
        use rcl_sys::RCUTILS_LOG_SEVERITY;

        if from == RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_UNSET as u32 as c_int {
            Ok(Self::Unset)
        } else if from == RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG as u32 as c_int {
            Ok(Self::Debug)
        } else if from == RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO as u32 as c_int {
            Ok(Self::Info)
        } else if from == RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN as u32 as c_int {
            Ok(Self::Warn)
        } else if from == RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR as u32 as c_int {
            Ok(Self::Error)
        } else if from == RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL as u32 as c_int {
            Ok(Self::Fatal)
        } else {
            Err(RclRustError::RuntimeError("cast error: LogSeverity").into())
        }
    }
}

pub(crate) static LOGGER_MUTEX: Lazy<ReentrantMutex<()>> = Lazy::new(|| ReentrantMutex::new(()));

#[derive(Debug)]
pub struct Logger {
    name: CString,
}

impl Logger {
    pub fn new(name: &str) -> Self {
        Self {
            name: CString::new(name).unwrap(),
        }
    }

    pub fn empty_name() -> Self {
        Self {
            name: CString::default(),
        }
    }

    pub fn log_common(&self, severity: LogSeverity, msg: &str, file: &str, line: u32) {
        let _guard = LOGGER_MUTEX.lock();

        unsafe {
            if !rcl_sys::g_rcutils_logging_initialized {
                rcl_sys::rcutils_logging_initialize()
                    .to_result()
                    .expect("rcutils_logging_initialize() should succeed");
            }
        }

        if !self.is_enable_for(severity) {
            return;
        }

        // TODO: How to get function name?
        let function_name = CString::new("()").unwrap();
        let file = CString::new(file).unwrap();
        let msg = CString::new(msg).unwrap();

        let logging_location = rcl_sys::rcutils_log_location_t {
            function_name: function_name.as_ptr(),
            file_name: file.as_ptr(),
            line_number: line.try_into().unwrap(),
        };

        unsafe {
            rcl_sys::rcutils_log(
                &logging_location,
                severity.into(),
                self.get_name_ptr(),
                msg.as_ptr(),
            );
        }
    }

    pub fn get_name(&self) -> &str {
        self.name.to_str().unwrap()
    }

    fn get_name_ptr(&self) -> *const c_char {
        self.name.as_ptr()
    }

    pub fn get_child(&self, suffix: &str) -> Self {
        if self.name.to_str().unwrap().is_empty() {
            Self::empty_name()
        } else {
            Self::new(&format!(
                "{}.{}",
                self.name.clone().into_string().unwrap(),
                suffix
            ))
        }
    }

    pub fn set_level(&self, level: LogSeverity) -> Result<()> {
        let _guard = LOGGER_MUTEX.lock();
        unsafe {
            rcl_sys::rcutils_logging_set_logger_level(self.get_name_ptr(), level.into()).to_result()
        }
    }

    pub fn get_level(&self) -> Result<LogSeverity> {
        LogSeverity::try_from(unsafe {
            let _guard = LOGGER_MUTEX.lock();
            rcl_sys::rcutils_logging_get_logger_level(self.get_name_ptr())
        })
        .with_context(|| format!("{:?}", self.name))
    }

    pub fn get_effective_level(&self) -> Result<LogSeverity> {
        LogSeverity::try_from(unsafe {
            let _guard = LOGGER_MUTEX.lock();
            rcl_sys::rcutils_logging_get_logger_effective_level(self.get_name_ptr())
        })
        .with_context(|| format!("{:?}", self.name))
    }

    pub fn is_enable_for(&self, severity: LogSeverity) -> bool {
        let _guard = LOGGER_MUTEX.lock();
        unsafe {
            rcl_sys::rcutils_logging_logger_is_enabled_for(self.get_name_ptr(), severity.into())
        }
    }
}

#[macro_export]
macro_rules! rclrust_debug {
    ($logger:expr, $($arg:tt)+) => {
        $logger.log_common($crate::log::LogSeverity::Debug, &format!($($arg)+), file!(), line!());
    };
}

#[macro_export]
macro_rules! rclrust_info {
    ($logger:expr, $($arg:tt)+) => {
        $logger.log_common($crate::log::LogSeverity::Info, &format!($($arg)+), file!(), line!());
    };
}

#[macro_export]
macro_rules! rclrust_warn {
    ($logger:expr, $($arg:tt)+) => {
        $logger.log_common($crate::log::LogSeverity::Warn, &format!($($arg)+), file!(), line!());
    };
}

#[macro_export]
macro_rules! rclrust_error {
    ($logger:expr, $($arg:tt)+) => {
        $logger.log_common($crate::log::LogSeverity::Error, &format!($($arg)+), file!(), line!());
    };
}

#[macro_export]
macro_rules! rclrust_fatal {
    ($logger:expr, $($arg:tt)+) => {
        $logger.log_common($crate::log::LogSeverity::Fatal, &format!($($arg)+), file!(), line!());
    };
}

#[cfg(test)]
mod test {
    use super::*;
    use std::sync::Mutex;

    static TEST_MUTEX: Lazy<Mutex<()>> = Lazy::new(|| Mutex::new(()));

    #[test]
    fn named_logger_init() -> Result<()> {
        let logger = Logger::new("test");
        assert_eq!(logger.get_name(), "test");
        Ok(())
    }

    #[test]
    fn get_child_logger() -> Result<()> {
        let logger = Logger::new("test");
        let child_logger = logger.get_child("child");
        assert_eq!(child_logger.get_name(), "test.child");
        let grandchild_logger = child_logger.get_child("child2");
        assert_eq!(grandchild_logger.get_name(), "test.child.child2");

        Ok(())
    }

    #[test]
    fn empty_logger_level() -> Result<()> {
        let _guard = TEST_MUTEX.lock();

        let logger = Logger::empty_name();

        logger.set_level(LogSeverity::Debug)?;
        assert_eq!(
            logger.get_level()?,
            LogSeverity::Debug,
            "get_level() should return LogSeverity::Debug"
        );
        assert_eq!(
            logger.get_effective_level()?,
            LogSeverity::Debug,
            "get_effective_level() should return LogSeverity::Debug"
        );

        // rollback
        logger.set_level(LogSeverity::Info)?;

        Ok(())
    }

    #[test]
    fn named_logger_level() -> Result<()> {
        let _guard = TEST_MUTEX.lock();

        let root_logger = Logger::empty_name();

        let logger = Logger::new("test");
        assert_eq!(logger.get_level()?, LogSeverity::Unset);
        assert_eq!(logger.get_effective_level()?, root_logger.get_level()?);

        logger.set_level(LogSeverity::Info)?;
        assert_eq!(logger.get_level()?, LogSeverity::Info);
        assert_eq!(logger.get_effective_level()?, LogSeverity::Info);

        // rollback
        logger.set_level(LogSeverity::Unset)?;

        Ok(())
    }

    #[test]
    fn empty_logger_is_enable_for() -> Result<()> {
        let _guard = TEST_MUTEX.lock();

        let logger = Logger::empty_name();

        logger.set_level(LogSeverity::Unset)?;
        assert!(logger.is_enable_for(LogSeverity::Fatal));
        assert!(logger.is_enable_for(LogSeverity::Error));
        assert!(logger.is_enable_for(LogSeverity::Warn));
        assert!(logger.is_enable_for(LogSeverity::Info));
        assert!(logger.is_enable_for(LogSeverity::Debug));

        logger.set_level(LogSeverity::Fatal)?;
        assert!(logger.is_enable_for(LogSeverity::Fatal));
        assert!(!logger.is_enable_for(LogSeverity::Error));
        assert!(!logger.is_enable_for(LogSeverity::Warn));
        assert!(!logger.is_enable_for(LogSeverity::Info));
        assert!(!logger.is_enable_for(LogSeverity::Debug));

        logger.set_level(LogSeverity::Error)?;
        assert!(logger.is_enable_for(LogSeverity::Fatal));
        assert!(logger.is_enable_for(LogSeverity::Error));
        assert!(!logger.is_enable_for(LogSeverity::Warn));
        assert!(!logger.is_enable_for(LogSeverity::Info));
        assert!(!logger.is_enable_for(LogSeverity::Debug));

        logger.set_level(LogSeverity::Warn)?;
        assert!(logger.is_enable_for(LogSeverity::Fatal));
        assert!(logger.is_enable_for(LogSeverity::Error));
        assert!(logger.is_enable_for(LogSeverity::Warn));
        assert!(!logger.is_enable_for(LogSeverity::Info));
        assert!(!logger.is_enable_for(LogSeverity::Debug));

        logger.set_level(LogSeverity::Info)?;
        assert!(logger.is_enable_for(LogSeverity::Fatal));
        assert!(logger.is_enable_for(LogSeverity::Error));
        assert!(logger.is_enable_for(LogSeverity::Warn));
        assert!(logger.is_enable_for(LogSeverity::Info));
        assert!(!logger.is_enable_for(LogSeverity::Debug));

        logger.set_level(LogSeverity::Debug)?;
        assert!(logger.is_enable_for(LogSeverity::Fatal));
        assert!(logger.is_enable_for(LogSeverity::Error));
        assert!(logger.is_enable_for(LogSeverity::Warn));
        assert!(logger.is_enable_for(LogSeverity::Info));
        assert!(logger.is_enable_for(LogSeverity::Debug));

        // rollback
        logger.set_level(LogSeverity::Info)?;

        Ok(())
    }
}
