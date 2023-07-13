use std::{
  io,
  net::{IpAddr, Ipv4Addr, SocketAddr},
};

use mio_06;
use log::{debug, error, info, trace, warn};
use socket2::{Domain, Protocol, SockAddr, Socket, Type};
use bytes::{Bytes, BytesMut};

use crate::{
  network::util::{
    get_local_multicast_ip_addrs, get_local_multicast_locators, get_local_unicast_locators,
  },
  structure::locator::Locator,
};

const MAX_MESSAGE_SIZE: usize = 64 * 1024; // This is max we can get from UDP.
const MESSAGE_BUFFER_ALLOCATION_CHUNK: usize = 256 * 1024; // must be >= MAX_MESSAGE_SIZE
static_assertions::const_assert!(MESSAGE_BUFFER_ALLOCATION_CHUNK > MAX_MESSAGE_SIZE);

/// Listens to messages coming to specified host port combination.
/// Only messages from added listen addressed are read when get_all_messages is
/// called.
#[derive(Debug)]
pub struct UDPListener {
  socket: mio_06::net::UdpSocket,
  receive_buffer: BytesMut,
  multicast_group: Option<Ipv4Addr>,
}

impl Drop for UDPListener {
  fn drop(&mut self) {
    if let Some(mcg) = self.multicast_group {
      self
        .socket
        .leave_multicast_v4(&mcg, &Ipv4Addr::UNSPECIFIED)
        .unwrap_or_else(|e| {
          error!("leave_multicast_group: {e:?}");
        });
    }
  }
}

impl UDPListener {
  fn new_listening_socket(
    host: &str,
    port: u16,
    reuse_addr: bool,
  ) -> io::Result<mio_06::net::UdpSocket> {
    let raw_socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))?;

    // We set ReuseAddr so that other DomainParticipants on this host can
    // bind to the same multicast address and port.
    // To have an effect on bind, this must be done before bind call, so must be
    // done below Rust std::net::UdpSocket level.
    if reuse_addr {
      raw_socket.set_reuse_address(true)?;
    }

    // MacOS requires this also
    #[cfg(not(any(target_os = "solaris", target_os = "illumos", windows)))]
    {
      if reuse_addr {
        raw_socket.set_reuse_port(true)?;
      }
    }

    let address = SocketAddr::new(
      host
        .parse()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?,
      port,
    );

    if let Err(e) = raw_socket.bind(&SockAddr::from(address)) {
      info!("new_socket - cannot bind socket: {e:?}");
      return Err(e);
    }

    let std_socket = std::net::UdpSocket::from(raw_socket);
    std_socket
      .set_nonblocking(true)
      .expect("Failed to set std socket to non blocking.");

    let mio_socket =
      mio_06::net::UdpSocket::from_socket(std_socket).expect("Unable to create mio socket");
    info!(
      "UDPListener: new socket with address {:?}",
      mio_socket.local_addr()
    );

    Ok(mio_socket)
  }

  pub fn to_locator_address(&self) -> io::Result<Vec<Locator>> {
    let local_port = self.socket.local_addr()?.port();

    match self.multicast_group {
      Some(_ipv4_addr) => Ok(get_local_multicast_locators(local_port)),
      None => Ok(get_local_unicast_locators(local_port)),
    }
  }

  pub fn new_unicast(host: &str, port: u16) -> io::Result<Self> {
    let mio_socket = Self::new_listening_socket(host, port, false)?;

    Ok(Self {
      socket: mio_socket,
      receive_buffer: BytesMut::with_capacity(MESSAGE_BUFFER_ALLOCATION_CHUNK),
      multicast_group: None,
    })
  }

  pub fn new_multicast(host: &str, port: u16, multicast_group: Ipv4Addr) -> io::Result<Self> {
    if !multicast_group.is_multicast() {
      return io::Result::Err(io::Error::new(
        io::ErrorKind::Other,
        "Not a multicast address",
      ));
    }

    let mio_socket = Self::new_listening_socket(host, port, true)?;

    for multicast_if_ipaddr in get_local_multicast_ip_addrs()? {
      match multicast_if_ipaddr {
        IpAddr::V4(a) => mio_socket
          .join_multicast_v4(&multicast_group, &a)
          .unwrap_or_else(|e| {
            warn!(
              "join_multicast_v4 failed: {:?}. multicast_group [{:?}] interface [{:?}]",
              e, multicast_group, a
            );
          }),
        IpAddr::V6(_a) => error!("UDPListener::new_multicast() not implemented for IpV6"), // TODO
      }
    }

    Ok(Self {
      socket: mio_socket,
      receive_buffer: BytesMut::with_capacity(MESSAGE_BUFFER_ALLOCATION_CHUNK),
      multicast_group: Some(multicast_group),
    })
  }

  pub fn mio_socket(&mut self) -> &mut mio_06::net::UdpSocket {
    &mut self.socket
  }

  #[cfg(test)]
  pub fn port(&self) -> u16 {
    match self.socket.local_addr() {
      Ok(add) => add.port(),
      _ => 0,
    }
  }

  // TODO: remove this. It is used only for tests.
  // We cannot read a single packet only, because we use edge-triggered polls.
  #[cfg(test)]
  pub fn get_message(&self) -> Vec<u8> {
    let mut message: Vec<u8> = vec![];
    let mut buf: [u8; MAX_MESSAGE_SIZE] = [0; MAX_MESSAGE_SIZE];
    match self.socket.recv(&mut buf) {
      Ok(nbytes) => {
        message = buf[..nbytes].to_vec();
        return message;
      }
      Err(e) => {
        debug!("UDPListener::get_message failed: {e:?}");
      }
    };
    message
  }

  /// Get all messages waiting in the socket.
  pub fn messages(&mut self) -> Vec<Bytes> {
    let mut messages = Vec::with_capacity(4);

    loop {
      // Loop invariant. Note that capacity() may be large, but .len() == 0.
      assert_eq!(self.receive_buffer.len(), 0);

      // Ensure that receive buffer has enough capacity for a message
      if self.receive_buffer.capacity() < MAX_MESSAGE_SIZE {
        self.receive_buffer = BytesMut::with_capacity(MESSAGE_BUFFER_ALLOCATION_CHUNK);
        debug!("ensure_receive_buffer_capacity - reallocated receive_buffer");
      }
      unsafe {
        // This is safe, because we just checked that there is enough capacity,
        // or allocated more.
        // We do not read undefined data, because the recv()
        // will overwrite this space and truncate the rest away.
        self.receive_buffer.set_len(MAX_MESSAGE_SIZE);
      }
      trace!(
        "ensure_receive_buffer_capacity - {} bytes left",
        self.receive_buffer.capacity()
      );
      let nbytes = match self.socket.recv(&mut self.receive_buffer) {
        Ok(n) => n,
        Err(e) => {
          self.receive_buffer.clear(); // since nothing was received
          if e.kind() == io::ErrorKind::WouldBlock {
            // This is the normal case.
          } else {
            warn!("socket recv() error: {e:?}");
          }
          // In any case, we stop trying and return.
          return messages;
        }
      };
      // Something was received.

      // Now, append some extra data to align the buffer end, so the next piece will
      // be aligned also. This assumes that the initial buffer was aligned to begin
      // with. This is because RTPS data is optimized to align to 4-byte boundaries.
      let unalign = self.receive_buffer.len() % 4;
      if unalign != 0 {
        self
          .receive_buffer
          .extend_from_slice(&[0xCC, 0xCC, 0xCC, 0xCC][..(4 - unalign)]);
        // Funny value 0xCC encourages a fast crash in case these bytes
        // are ever accessed, as they should not.
      }

      // Now split away the used portion.
      let mut message = self.receive_buffer.split_to(self.receive_buffer.len());
      message.truncate(nbytes); // discard (hide) padding
      messages.push(Bytes::from(message)); // freeze bytes and push
    } // loop

    //unreachable!(); // But why does this cause a warning? (rustc 1.66.0)
    // Answer: https://github.com/rust-lang/rust/issues/46500
  }

  #[cfg(test)] // normally done in .drop()
  pub fn leave_multicast(&self, address: &Ipv4Addr) -> io::Result<()> {
    if address.is_multicast() {
      return self
        .socket
        .leave_multicast_v4(address, &Ipv4Addr::UNSPECIFIED);
    }
    io::Result::Err(io::Error::new(
      io::ErrorKind::Other,
      "Not a multicast address",
    ))
  }
}

#[cfg(test)]
mod tests {
  //use std::os::unix::io::AsRawFd;
  //use nix::sys::socket::setsockopt;
  //use nix::sys::socket::sockopt::IpMulticastLoop;
  use std::{thread, time};

  use super::*;
  use crate::network::udp_sender::*;

  #[test]
  fn udpl_single_address() {
    let listener = UDPListener::new_unicast("127.0.0.1", 10001).unwrap();
    let sender = UDPSender::new_with_random_port().expect("failed to create UDPSender");

    let data: Vec<u8> = vec![0, 1, 2, 3, 4];

    let addrs = vec![SocketAddr::new("127.0.0.1".parse().unwrap(), 10001)];
    sender.send_to_all(&data, &addrs);

    let rec_data = listener.get_message();

    assert_eq!(rec_data.len(), 5); // It appears that this test may randomly fail.
    assert_eq!(rec_data, data);
  }

  #[test]
  fn udpl_multicast_address() {
    let listener =
      UDPListener::new_multicast("0.0.0.0", 10002, Ipv4Addr::new(239, 255, 0, 1)).unwrap();
    let sender = UDPSender::new_with_random_port().unwrap();

    //setsockopt(sender.socket.as_raw_fd(), IpMulticastLoop, &true)
    //  .expect("Unable set IpMulticastLoop option on socket");

    let data: Vec<u8> = vec![2, 4, 6];

    sender
      .send_multicast(&data, Ipv4Addr::new(239, 255, 0, 1), 10002)
      .expect("Failed to send multicast");

    thread::sleep(time::Duration::from_secs(1));

    let rec_data = listener.get_message();

    listener
      .leave_multicast(&Ipv4Addr::new(239, 255, 0, 1))
      .unwrap();

    assert_eq!(rec_data.len(), 3);
    assert_eq!(rec_data, data);
  }
}
