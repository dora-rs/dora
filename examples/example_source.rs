use eyre::Context;
use futures::SinkExt;
use std::{collections::BTreeMap, time::Duration};
use tokio_util::codec::{Framed, LengthDelimitedCodec};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let server_socket_addr = std::env::var("SERVER_SOCKET_ADDR")
        .wrap_err("failed to read `SERVER_SOCKET_ADDR` environment variable")?;

    let tcp = tokio::net::TcpStream::connect(&server_socket_addr)
        .await
        .with_context(|| {
            format!("failed to open TCP connection to server at {server_socket_addr}")
        })?;
    let mut framed = Framed::new(tcp, LengthDelimitedCodec::new());

    let mut i: u8 = 0;
    loop {
        let mut outputs: BTreeMap<_, Vec<u8>> = Default::default();
        outputs.insert("A", vec![i]);
        if i % 3 == 0 {
            outputs.insert("fizz", vec![]);
        }
        if i % 5 == 0 {
            outputs.insert("buzz", vec![]);
        }
        outputs.insert("Squared", vec![i.wrapping_mul(i)]);

        let serialized = bincode::serialize(&outputs).wrap_err("failed to serialize output")?;

        framed
            .send(serialized.into())
            .await
            .wrap_err("failed to send output")?;

        std::thread::sleep(Duration::from_millis(100));
        i = i.wrapping_add(1);
    }
}
