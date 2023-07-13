use crate::dds::qos::HasQoSPolicy;

/// Intended to somewhat describe DDS 2.2.2.1.1 Entity Class
pub trait DDSEntity: HasQoSPolicy {}

// But we are not entirely sure what this does.
// TODO: Consider removing this, as it seems to serve no useful purpose.
