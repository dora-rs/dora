use anyhow::Context;
use futures::SinkExt;
use futures::stream::StreamExt;
use gstreamer::prelude::*;
use gstreamer_app::AppSrc;
use gstreamer_webrtc::{WebRTCICEConnectionState, WebRTCSessionDescription};
use log::{error, info, warn};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use warp::ws::{Message, WebSocket};

use crate::peer_connection::PeerConnection;
use crate::signaling::SignalingMessage;

#[derive(Debug, Clone)]
pub struct WebRTCServer {
    peers: Arc<Mutex<HashMap<String, PeerConnection>>>,
    _frame_sender: mpsc::Sender<Vec<u8>>,
}

impl WebRTCServer {
    pub fn new(frame_sender: mpsc::Sender<Vec<u8>>) -> Self {
        Self {
            peers: Arc::new(Mutex::new(HashMap::new())),
            _frame_sender: frame_sender,
        }
    }

    pub async fn handle_websocket(&self, ws: WebSocket, id: String) {
        let (ws_sender, mut ws_receiver) = ws.split();
        let ws_sender = Arc::new(Mutex::new(ws_sender));

        while let Some(result) = ws_receiver.next().await {
            match result {
                Ok(msg) => {
                    if let Ok(text) = msg.to_str() {
                        if let Err(e) = self
                            .handle_signaling_message(text, &id, ws_sender.clone())
                            .await
                        {
                            error!("Error handling signaling message: {}", e);
                        }
                    }
                }
                Err(e) => {
                    error!("WebSocket error: {}", e);
                    break;
                }
            }
        }

        self.remove_peer(&id);
        info!("Client {} disconnected", id);
    }

    async fn handle_signaling_message(
        &self,
        msg: &str,
        peer_id: &str,
        ws_sender: Arc<Mutex<futures::stream::SplitSink<WebSocket, Message>>>,
    ) -> anyhow::Result<()> {
        // Parse JSON with detailed error handling
        let message: SignalingMessage = match serde_json::from_str(msg) {
            Ok(m) => m,
            Err(e) => {
                // Log the raw message for debugging
                error!("Failed to deserialize signaling message. Error: {}", e);
                error!("Raw message: {}", msg);

                // Try to parse as generic JSON to provide more context
                if let Ok(json_value) = serde_json::from_str::<serde_json::Value>(msg) {
                    error!("Parsed JSON structure: {:#?}", json_value);

                    // Check the type field
                    if let Some(msg_type) = json_value.get("type") {
                        error!("Message type field: {:?}", msg_type);
                    } else {
                        error!("Message missing 'type' field");
                    }

                    // For offer/answer messages, check sdp field
                    if let Some(sdp) = json_value.get("sdp") {
                        error!(
                            "SDP field type: {}",
                            if sdp.is_string() {
                                "string"
                            } else {
                                "non-string"
                            }
                        );
                    }

                    // For ICE messages, check candidate field
                    if let Some(candidate) = json_value.get("candidate") {
                        error!(
                            "Candidate field type: {}",
                            if candidate.is_string() {
                                "string"
                            } else {
                                "non-string"
                            }
                        );
                    }
                }

                return Err(anyhow::anyhow!(
                    "Failed to deserialize signaling message: {}",
                    e
                ));
            }
        };

        match message {
            SignalingMessage::Offer { sdp, .. } => {
                info!("Received offer from {}", peer_id);
                let _answer_sdp = self.handle_offer(&sdp, peer_id, ws_sender).await?;
            }
            SignalingMessage::Ice { candidate } => {
                self.add_ice_candidate(peer_id, &candidate.candidate, candidate.sdp_mline_index)?;
            }
            _ => {
                warn!("Unexpected message type");
            }
        }

        Ok(())
    }

    async fn handle_offer(
        &self,
        sdp: &str,
        peer_id: &str,
        ws_sender: Arc<Mutex<futures::stream::SplitSink<WebSocket, Message>>>,
    ) -> anyhow::Result<String> {
        let pipeline = self.create_pipeline(peer_id, ws_sender.clone())?;

        let webrtcbin = pipeline
            .by_name("webrtc")
            .context("Failed to get webrtcbin")?;

        let appsrc = pipeline
            .by_name("videosrc")
            .and_then(|e| e.dynamic_cast::<AppSrc>().ok())
            .context("Failed to get appsrc")?;

        // Configure appsrc
        appsrc.set_property("is-live", true);
        appsrc.set_property("format", gstreamer::Format::Time);

        // Set pipeline to READY state before setting remote description
        pipeline.set_state(gstreamer::State::Ready)?;

        // Wait for state change to complete
        let (state_change_return, _, _) = pipeline.state(gstreamer::ClockTime::from_seconds(5));
        if state_change_return.is_err() {
            return Err(anyhow::anyhow!("Failed to set pipeline to READY state"));
        }

        let ret = gstreamer_sdp::SDPMessage::parse_buffer(sdp.as_bytes())
            .map_err(|_| anyhow::anyhow!("Failed to parse SDP"))?;

        let offer = WebRTCSessionDescription::new(gstreamer_webrtc::WebRTCSDPType::Offer, ret);

        let promise = gstreamer::Promise::new();
        webrtcbin.emit_by_name::<()>("set-remote-description", &[&offer, &promise]);

        // Wait for set-remote-description to complete
        let result = promise.wait();
        match result {
            gstreamer::PromiseResult::Replied => {
                // Check if there was an error in the reply
                if let Some(reply) = promise.get_reply() {
                    if let Ok(error_value) = reply.value("error") {
                        if let Ok(error) = error_value.get::<glib::Error>() {
                            error!("set-remote-description error: {}", error);
                            return Err(anyhow::anyhow!(
                                "Failed to set remote description: {}",
                                error
                            ));
                        }
                    }
                }
            }
            _ => {
                error!("Failed to set remote description: {:?}", result);
                return Err(anyhow::anyhow!("Failed to set remote description"));
            }
        }

        // Additional delay to ensure state is propagated
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Store peer first to ensure it can receive frames
        let peer = PeerConnection::new(
            peer_id.to_string(),
            pipeline.clone(),
            webrtcbin.clone(),
            appsrc.clone(),
        );
        self.peers.lock().unwrap().insert(peer_id.to_string(), peer);

        // Now set pipeline to PLAYING state before creating answer
        pipeline.set_state(gstreamer::State::Playing)?;

        // Wait for state change to complete
        let (state_change_return, _, _) = pipeline.state(gstreamer::ClockTime::from_seconds(5));
        if state_change_return.is_err() {
            warn!(
                "Pipeline state change to PLAYING not fully successful: {:?}",
                state_change_return
            );
        }

        // Give pipeline time to initialize
        std::thread::sleep(std::time::Duration::from_millis(200));

        // Connect to pad-added signal to ensure pipeline is ready
        webrtcbin.connect("pad-added", false, move |values| {
            let _webrtc = match values[0].get::<gstreamer::Element>() {
                Ok(elem) => elem,
                Err(_) => {
                    error!("Failed to get webrtc element from values");
                    return None;
                }
            };

            let _pad = match values[1].get::<gstreamer::Pad>() {
                Ok(p) => p,
                Err(_) => {
                    error!("Failed to get pad from values");
                    return None;
                }
            };

            None
        });

        // Use promise without change func first
        let promise = gstreamer::Promise::new();

        webrtcbin.emit_by_name::<()>("create-answer", &[&None::<gstreamer::Structure>, &promise]);

        // Wait for promise to complete
        let reply = promise.wait();

        // Small delay to ensure the answer is fully processed
        std::thread::sleep(std::time::Duration::from_millis(100));

        // Get the answer from promise reply
        match reply {
            gstreamer::PromiseResult::Replied => {
                if let Some(reply_struct) = promise.get_reply() {
                    // Check for error in reply
                    if let Ok(error_value) = reply_struct.value("error") {
                        if let Ok(error) = error_value.get::<glib::Error>() {
                            error!("create-answer error: {}", error);
                        }
                    }

                    // The answer should be in the reply
                    if let Ok(answer_value) =
                        reply_struct.get::<gstreamer_webrtc::WebRTCSessionDescription>("answer")
                    {
                        // Set local description
                        let set_promise = gstreamer::Promise::new();
                        webrtcbin.emit_by_name::<()>(
                            "set-local-description",
                            &[&answer_value, &set_promise],
                        );
                        let _ = set_promise.wait();

                        let answer_sdp = answer_value.sdp().to_string();

                        let msg = SignalingMessage::Answer { sdp: answer_sdp };

                        if let Ok(json) = serde_json::to_string(&msg) {
                            let ws_sender_clone = ws_sender.clone();
                            tokio::task::spawn_blocking(move || {
                                let rt = tokio::runtime::Handle::current();
                                rt.block_on(async move {
                                    if let Ok(mut sender) = ws_sender_clone.lock() {
                                        let _ = sender.send(Message::text(json)).await;
                                    }
                                });
                            });
                        }
                    } else {
                        error!("No answer in reply structure");

                        // Try to get local-description as fallback
                        if let Some(answer) = webrtcbin.property::<Option<
                            gstreamer_webrtc::WebRTCSessionDescription,
                        >>(
                            "local-description"
                        ) {
                            let answer_sdp = answer.sdp().to_string();

                            let msg = SignalingMessage::Answer { sdp: answer_sdp };

                            if let Ok(json) = serde_json::to_string(&msg) {
                                let ws_sender_clone = ws_sender.clone();
                                tokio::task::spawn_blocking(move || {
                                    let rt = tokio::runtime::Handle::current();
                                    rt.block_on(async move {
                                        if let Ok(mut sender) = ws_sender_clone.lock() {
                                            let _ = sender.send(Message::text(json)).await;
                                        }
                                    });
                                });
                            }
                        }
                    }
                }
            }
            _ => {
                error!("Promise failed or was interrupted");
            }
        }

        Ok(String::new())
    }

    fn create_pipeline(
        &self,
        peer_id: &str,
        ws_sender: Arc<Mutex<futures::stream::SplitSink<WebSocket, Message>>>,
    ) -> anyhow::Result<gstreamer::Pipeline> {
        // Accept any resolution and framerate from input
        // Initial caps will be set dynamically when first frame arrives
        let pipeline_str = format!(
            "appsrc name=videosrc \
             is-live=true format=time do-timestamp=true ! \
             videoconvert ! \
             vp8enc deadline=1 ! \
             rtpvp8pay ! \
             application/x-rtp,media=video,encoding-name=VP8,payload=96 ! \
             webrtcbin name=webrtc bundle-policy=max-bundle"
        );

        let pipeline = gstreamer::parse::launch(&pipeline_str)?
            .dynamic_cast::<gstreamer::Pipeline>()
            .map_err(|_| anyhow::anyhow!("Failed to create pipeline"))?;

        // Monitor pipeline messages
        let bus = pipeline.bus().unwrap();
        let peer_id_for_bus = peer_id.to_string();
        std::thread::spawn(move || {
            for msg in bus.iter_timed(gstreamer::ClockTime::NONE) {
                use gstreamer::MessageView;
                match msg.view() {
                    MessageView::StateChanged(_) => {}
                    MessageView::Error(err) => {
                        error!(
                            "Pipeline {} error: {} ({})",
                            peer_id_for_bus,
                            err.error(),
                            err.debug().unwrap_or_default()
                        );
                    }
                    MessageView::Warning(warn) => {
                        warn!(
                            "Pipeline {} warning: {} ({})",
                            peer_id_for_bus,
                            warn.error(),
                            warn.debug().unwrap_or_default()
                        );
                    }
                    MessageView::Eos(_) => {
                        break;
                    }
                    _ => {}
                }
            }
        });

        let webrtcbin = pipeline
            .by_name("webrtc")
            .context("Failed to get webrtcbin")?;

        webrtcbin.set_property_from_str("stun-server", "stun://stun.l.google.com:19302");

        let ws_sender_clone = ws_sender.clone();
        webrtcbin.connect("on-ice-candidate", false, move |values| {
            // Handle potential errors in callback to avoid panic
            let _webrtc = match values[0].get::<gstreamer::Element>() {
                Ok(elem) => elem,
                Err(_) => {
                    error!("Failed to get webrtc element from values");
                    return None;
                }
            };

            let mline_index = match values[1].get::<u32>() {
                Ok(idx) => idx,
                Err(_) => {
                    error!("Failed to get mline_index from values");
                    return None;
                }
            };

            let candidate = match values[2].get::<String>() {
                Ok(cand) => cand,
                Err(_) => {
                    error!("Failed to get candidate from values");
                    return None;
                }
            };

            let msg = SignalingMessage::Ice {
                candidate: crate::signaling::IceCandidate {
                    candidate,
                    sdp_mline_index: mline_index,
                    sdp_mid: None,
                },
            };

            if let Ok(json) = serde_json::to_string(&msg) {
                let ws_sender = ws_sender_clone.clone();

                // Use a dedicated runtime for sending messages from GStreamer threads
                std::thread::spawn(move || {
                    // Create a small runtime just for this send operation
                    let rt = tokio::runtime::Builder::new_current_thread()
                        .enable_all()
                        .build();

                    if let Ok(rt) = rt {
                        rt.block_on(async move {
                            if let Ok(mut sender) = ws_sender.lock() {
                                let _ = sender.send(Message::text(json)).await;
                            }
                        });
                    }
                });
            }

            None
        });

        let peer_id_clone = peer_id.to_string();
        let _ws_sender_clone = ws_sender.clone();
        webrtcbin.connect("notify::ice-connection-state", false, move |values| {
            let _webrtc = match values[0].get::<gstreamer::Element>() {
                Ok(elem) => elem,
                Err(_) => {
                    error!("Failed to get webrtc element from values");
                    return None;
                }
            };

            let state = _webrtc.property::<WebRTCICEConnectionState>("ice-connection-state");
            info!("ICE connection state for {}: {:?}", peer_id_clone, state);

            // Also check connection state
            let conn_state =
                _webrtc.property::<gstreamer_webrtc::WebRTCPeerConnectionState>("connection-state");
            info!(
                "Peer connection state for {}: {:?}",
                peer_id_clone, conn_state
            );
            None
        });

        Ok(pipeline)
    }

    fn add_ice_candidate(
        &self,
        peer_id: &str,
        candidate: &str,
        mline_index: u32,
    ) -> anyhow::Result<()> {
        let peers = self.peers.lock().unwrap();
        if let Some(peer) = peers.get(peer_id) {
            peer.webrtcbin
                .emit_by_name::<()>("add-ice-candidate", &[&mline_index, &candidate]);
        } else {
            warn!("Peer {} not found when adding ICE candidate", peer_id);
        }
        Ok(())
    }

    fn remove_peer(&self, peer_id: &str) {
        let mut peers = self.peers.lock().unwrap();
        if let Some(peer) = peers.remove(peer_id) {
            peer.shutdown();
        }
    }

    pub fn send_frame_to_peers(&self, frame_data: &[u8]) {
        let peers = self.peers.lock().unwrap();
        for (peer_id, peer) in peers.iter() {
            if let Err(e) = peer.send_frame(frame_data) {
                warn!("Failed to push buffer to peer {}: {}", peer_id, e);
            }
        }
    }
}
