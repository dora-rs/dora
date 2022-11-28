use std::{
    io::{Read, Write},
    net::{Ipv4Addr, TcpStream},
};

use dora_core::{config::NodeId, topics::DORA_DAEMON_PORT_DEFAULT};
use eyre::{eyre, Context};

pub struct DaemonConnection {
    stream: TcpStream,
}

impl DaemonConnection {
    pub fn init(node_id: NodeId) -> eyre::Result<Self> {
        let localhost = Ipv4Addr::new(127, 0, 0, 1);
        let mut stream = TcpStream::connect((localhost, DORA_DAEMON_PORT_DEFAULT))
            .wrap_err("failed to connect to dora-daemon")?;

        tcp_send(
            &mut stream,
            &dora_core::daemon_messages::Request::Register { node_id },
        )
        .wrap_err("failed to send register request to dora-daemon")?;

        match tcp_receive(&mut stream)
            .wrap_err("failed to receive register reply from dora-daemon")?
        {
            dora_core::daemon_messages::Reply::RegisterResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to register node with dora-daemon")?,
        }

        Ok(Self { stream })
    }
}

fn tcp_send(
    connection: &mut TcpStream,
    request: &dora_core::daemon_messages::Request,
) -> std::io::Result<()> {
    let serialized = serde_json::to_vec(request)?;

    let len_raw = (serialized.len() as u64).to_le_bytes();
    connection.write_all(&len_raw)?;
    connection.write_all(&serialized)?;
    Ok(())
}

fn tcp_receive(connection: &mut TcpStream) -> std::io::Result<dora_core::daemon_messages::Reply> {
    let reply_len = {
        let mut raw = [0; 8];
        connection.read_exact(&mut raw)?;
        u64::from_le_bytes(raw) as usize
    };
    let mut reply_raw = vec![0; reply_len];
    connection.read_exact(&mut reply_raw)?;

    let reply = serde_json::from_slice(&reply_raw)?;

    Ok(reply)
}
