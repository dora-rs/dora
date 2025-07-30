use log::info;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;

use crate::webrtc_server::WebRTCServer;

#[derive(Clone)]
pub struct VideoSourceManager {
    sources: Arc<Mutex<HashMap<String, VideoSource>>>,
}

#[derive(Clone)]
struct VideoSource {
    _video_id: String,
    server: Arc<WebRTCServer>,
    frame_sender: mpsc::Sender<Vec<u8>>,
}

impl VideoSourceManager {
    pub fn new() -> Self {
        Self {
            sources: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn get_or_create_source(
        &self,
        video_id: &str,
    ) -> (Arc<WebRTCServer>, mpsc::Sender<Vec<u8>>) {
        let mut sources = self.sources.lock().unwrap();

        if let Some(source) = sources.get(video_id) {
            (source.server.clone(), source.frame_sender.clone())
        } else {
            info!("Creating new video source: {}", video_id);

            let (frame_sender, mut frame_receiver) = mpsc::channel::<Vec<u8>>(1);
            let frame_sender_clone = frame_sender.clone();
            let server = Arc::new(WebRTCServer::new(frame_sender_clone));

            let server_clone = server.clone();
            let video_id_clone = video_id.to_string();
            tokio::spawn(async move {
                while let Some(frame) = frame_receiver.recv().await {
                    server_clone.send_frame_to_peers(&frame);
                }
                info!("Frame receiver for {} stopped", video_id_clone);
            });

            let source = VideoSource {
                _video_id: video_id.to_string(),
                server: server.clone(),
                frame_sender: frame_sender.clone(),
            };

            sources.insert(video_id.to_string(), source);

            (server, frame_sender)
        }
    }

    #[allow(dead_code)]
    pub fn get_source(&self, video_id: &str) -> Option<Arc<WebRTCServer>> {
        let sources = self.sources.lock().unwrap();
        sources.get(video_id).map(|s| s.server.clone())
    }

    #[allow(dead_code)]
    pub fn remove_source(&self, video_id: &str) {
        let mut sources = self.sources.lock().unwrap();
        if sources.remove(video_id).is_some() {
            info!("Removed video source: {}", video_id);
        }
    }
}
