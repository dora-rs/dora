use std::io;

use async_trait::async_trait;
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

use crate::{Transport, transport::AsyncTransport};

pub struct FramedTransport<T> {
    inner: T,
}
impl<T> FramedTransport<T> {
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    pub fn inner_mut(&mut self) -> &mut T {
        &mut self.inner
    }

    pub fn into_inner(self) -> T {
        self.inner
    }
}

impl<T> From<T> for FramedTransport<T> {
    fn from(value: T) -> Self {
        Self::new(value)
    }
}

impl<T> Transport<[u8], Vec<u8>> for FramedTransport<T>
where
    T: io::Read + io::Write,
{
    fn send(&mut self, item: &[u8]) -> io::Result<()> {
        self.inner.write_all(&(item.len() as u64).to_le_bytes())?;
        self.inner.write_all(item)?;
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<Vec<u8>>> {
        let mut len_buf = [0u8; 8];
        match self.inner.read_exact(&mut len_buf) {
            Ok(_) => {}
            // TODO: verify that this is the correct way to detect closed transport
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e),
        }
        let len = u64::from_le_bytes(len_buf) as usize;
        let mut buf = vec![0u8; len];
        self.inner.read_exact(&mut buf)?;
        Ok(Some(buf))
    }
}

#[async_trait]
impl<T> AsyncTransport<[u8], Vec<u8>> for FramedTransport<T>
where
    T: AsyncRead + AsyncWrite + Send + Unpin,
{
    async fn send(&mut self, item: &[u8]) -> io::Result<()> {
        self.inner
            .write_all(&(item.len() as u64).to_le_bytes())
            .await?;
        self.inner.write_all(item).await?;
        Ok(())
    }

    async fn receive(&mut self) -> io::Result<Option<Vec<u8>>> {
        let mut len_buf = [0u8; 8];
        match self.inner.read_exact(&mut len_buf).await {
            Ok(_) => {}
            // TODO: verify that this is the correct way to detect closed transport
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e),
        }
        let len = u64::from_le_bytes(len_buf) as usize;
        let mut buf = vec![0u8; len];
        self.inner.read_exact(&mut buf).await?;
        Ok(Some(buf))
    }
}
