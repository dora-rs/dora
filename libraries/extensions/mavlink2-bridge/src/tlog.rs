//! Replay a recorded `.tlog` file as a read-only MAVLink transport.
//!
//! A `.tlog` is what QGroundControl, Mission Planner and MAVProxy save from
//! a telemetry link: every MAVLink frame received, each preceded by an
//! 8-byte big-endian timestamp in microseconds. [`TlogConnection`] reads
//! the records with [`mavlink_tlog::TlogReader`], which skips damaged
//! records instead of stopping, and hands each frame to `mavlink` to decode.
//!
//! Frames are released at the pace they were recorded (scaled by `speed`),
//! so a dataflow sees the same timing it would have seen on the live link.
//! A single `recv` never waits longer than [`MAX_WAIT`], so a long gap in
//! the log does not keep the bridge's reader thread from noticing a
//! shutdown. Once the log is exhausted the link goes quiet: `recv` keeps
//! returning `WouldBlock` like an idle TCP link, until the node is stopped.

use std::fs::File;
use std::io::{self, BufReader};
use std::path::PathBuf;
use std::sync::{Mutex, PoisonError};
use std::time::{Duration, Instant};

use mavlink::error::{MessageReadError, MessageWriteError};
use mavlink::peek_reader::PeekReader;
use mavlink::{MAVLinkMessageRaw, MavConnection, MavHeader, MavlinkVersion, Message};
use mavlink_tlog::{OwnedRecord, TlogReader};

/// Longest a single `recv` blocks. Matches the 100 ms read timeout of
/// mavlink's TCP transport, which the bridge node's shutdown path relies on.
pub const MAX_WAIT: Duration = Duration::from_millis(100);

/// A `tlog://` endpoint: the file to replay and the playback speed.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct TlogConfig {
    pub(crate) path: PathBuf,
    /// Playback speed relative to the recording; `0.0` replays unpaced.
    pub(crate) speed: f64,
}

impl std::fmt::Display for TlogConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "tlog:{} (speed {})", self.path.display(), self.speed)
    }
}

/// A recorded `.tlog` played back through the [`MavConnection`] interface.
///
/// Frames are decoded as MAVLink 1 or 2, whichever each one was recorded
/// in. Frames the dialect cannot decode (for example ArduPilot-specific
/// messages in a log replayed with the common dialect) are skipped, as
/// mavlink's own file transport does. Sending is not supported.
pub struct TlogConnection {
    replay: Mutex<Replay>,
    speed: f64,
    version: MavlinkVersion,
    allow_any_version: bool,
}

struct Replay {
    reader: TlogReader<BufReader<File>>,
    /// The next record, read but not yet due.
    pending: Option<OwnedRecord>,
    /// Wall-clock instant and log timestamp of the first record.
    origin: Option<(Instant, u64)>,
}

impl TlogConnection {
    pub(crate) fn open(config: &TlogConfig) -> io::Result<Self> {
        let file = File::open(&config.path)?;
        Ok(Self {
            replay: Mutex::new(Replay {
                reader: TlogReader::new(BufReader::new(file)),
                pending: None,
                origin: None,
            }),
            speed: config.speed,
            version: MavlinkVersion::V2,
            allow_any_version: true,
        })
    }

    /// The next due frame, or `WouldBlock` once [`MAX_WAIT`] passed without
    /// one (or right away when `block` is false).
    fn next_frame(&self, block: bool) -> Result<Vec<u8>, MessageReadError> {
        let mut replay = self.replay.lock().unwrap_or_else(PoisonError::into_inner);
        let record = match replay.pending.take() {
            Some(record) => record,
            None => match replay.reader.next() {
                Some(Ok(record)) => record,
                Some(Err(e)) => return Err(MessageReadError::Io(e)),
                None => {
                    drop(replay);
                    if block {
                        std::thread::sleep(MAX_WAIT);
                    }
                    return Err(would_block("end of tlog replay"));
                }
            },
        };
        let wait = replay.wait_for(record.timestamp_us, self.speed);
        if wait.is_zero() {
            return Ok(record.frame);
        }
        if block && wait <= MAX_WAIT {
            drop(replay);
            std::thread::sleep(wait);
            return Ok(record.frame);
        }
        replay.pending = Some(record);
        drop(replay);
        if block {
            std::thread::sleep(MAX_WAIT);
        }
        Err(would_block("next tlog record is not due yet"))
    }

    fn read<M: Message>(&self, block: bool) -> Result<(MavHeader, M), MessageReadError> {
        loop {
            let frame = self.next_frame(block)?;
            if let Ok(message) = mavlink::read_any_msg(&mut PeekReader::new(frame.as_slice())) {
                return Ok(message);
            }
        }
    }
}

impl Replay {
    /// How long until a record stamped `timestamp_us` is due at `speed`.
    fn wait_for(&mut self, timestamp_us: u64, speed: f64) -> Duration {
        if speed == 0.0 {
            return Duration::ZERO;
        }
        let (start, first_us) = *self
            .origin
            .get_or_insert_with(|| (Instant::now(), timestamp_us));
        // A tiny `speed` can push the due time past what `Duration` or
        // `Instant` can hold; such a record is simply never due.
        let offset_secs = timestamp_us.saturating_sub(first_us) as f64 / 1e6 / speed;
        Duration::try_from_secs_f64(offset_secs)
            .ok()
            .and_then(|offset| start.checked_add(offset))
            .map_or(Duration::MAX, |due| {
                due.saturating_duration_since(Instant::now())
            })
    }
}

fn would_block(reason: &str) -> MessageReadError {
    MessageReadError::Io(io::Error::new(io::ErrorKind::WouldBlock, reason))
}

impl<M: Message> MavConnection<M> for TlogConnection {
    fn recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        self.read(true)
    }

    fn recv_raw(&self) -> Result<MAVLinkMessageRaw, MessageReadError> {
        loop {
            let frame = self.next_frame(true)?;
            if let Ok(raw) =
                mavlink::read_any_raw_message::<M, _>(&mut PeekReader::new(frame.as_slice()))
            {
                return Ok(raw);
            }
        }
    }

    fn try_recv(&self) -> Result<(MavHeader, M), MessageReadError> {
        self.read(false)
    }

    fn send(&self, _header: &MavHeader, _data: &M) -> Result<usize, MessageWriteError> {
        Err(MessageWriteError::Io(io::Error::new(
            io::ErrorKind::Unsupported,
            "a tlog replay is read-only",
        )))
    }

    fn set_protocol_version(&mut self, version: MavlinkVersion) {
        self.version = version;
    }

    fn protocol_version(&self) -> MavlinkVersion {
        self.version
    }

    fn set_allow_recv_any_version(&mut self, allow: bool) {
        self.allow_any_version = allow;
    }

    fn allow_recv_any_version(&self) -> bool {
        self.allow_any_version
    }
}
