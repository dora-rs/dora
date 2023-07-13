use std::{
  io,
  net::{IpAddr, SocketAddr},
};

use if_addrs::Interface;
#[allow(unused_imports)]
use log::{debug, error, info, trace};

use crate::structure::locator::Locator;

pub fn get_local_multicast_locators(port: u16) -> Vec<Locator> {
  let saddr = SocketAddr::new("239.255.0.1".parse().unwrap(), port);
  vec![Locator::from(saddr)]
}

pub fn get_local_unicast_locators(port: u16) -> Vec<Locator> {
  match if_addrs::get_if_addrs() {
    Ok(ifaces) => ifaces
      .iter()
      .filter(|ip| !ip.is_loopback())
      .map(|ip| Locator::from(SocketAddr::new(ip.ip(), port)))
      .collect(),
    Err(e) => {
      error!(
        "Cannot get local network interfaces: get_if_addrs() : {:?}",
        e
      );
      vec![]
    }
  }
}

// Enumerates local ip interfaces that we use for multicasting.
// This is used to set up senders and listeners.
//
// TODO: Check that the interface actually has multicast enabled.
// Now we just skip loopback.
// Could use e.g. "interfaces" crate to do this.
pub fn get_local_multicast_ip_addrs() -> io::Result<Vec<IpAddr>> {
  let ifs = if_addrs::get_if_addrs()?;
  Ok(
    ifs
      .iter()
      .filter(|ifaddr| !ifaddr.is_loopback())
      .map(Interface::ip)
      .collect(),
  )
}
