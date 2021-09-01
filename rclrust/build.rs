#[allow(unused_imports)]
use std::env;

#[cfg(not(any(feature = "foxy", feature = "galactic", feature = "rolling")))]
compile_error!("Any distribution feature should be specified");

fn main() {
    #[cfg(feature = "foxy")]
    assert_eq!(env::var("ROS_DISTRO"), Ok("foxy".to_string()));
    #[cfg(feature = "galactic")]
    assert_eq!(env::var("ROS_DISTRO"), Ok("galactic".to_string()));
    #[cfg(feature = "rolling")]
    assert_eq!(env::var("ROS_DISTRO"), Ok("rolling".to_string()));
}
