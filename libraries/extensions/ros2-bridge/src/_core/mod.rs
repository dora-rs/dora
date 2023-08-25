pub use widestring;

pub mod sequence;
pub mod string;
pub mod traits;

pub use sequence::{FFISeq, OwnedFFISeq, RefFFISeq};
pub use string::{FFIString, FFIWString, OwnedFFIString, OwnedFFIWString};
pub use traits::{ActionT, FFIFromRust, FFIToRust, InternalDefault, MessageT, ServiceT};

pub type ServiceRequestRaw<Srv> = <<Srv as ServiceT>::Request as MessageT>::Raw;
pub type ServiceResponseRaw<Srv> = <<Srv as ServiceT>::Response as MessageT>::Raw;
