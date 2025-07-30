use arrow::array::UInt8Array;
use dora_node_api::{DoraNode, Event};
use futures::stream::StreamExt;
use log::info;
use std::env;
use std::sync::Arc;
use uuid::Uuid;
use warp::Filter;

mod peer_connection;
mod signaling;
mod video_source_manager;
mod webrtc_server;

use video_source_manager::VideoSourceManager;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    env_logger::init();
    gstreamer::init()?;

    let source_manager = Arc::new(VideoSourceManager::new());

    // WebSocket route with video_id parameter
    let source_manager_clone = source_manager.clone();
    let websocket_route =
        warp::path!(String)
            .and(warp::ws())
            .map(move |video_id: String, ws: warp::ws::Ws| {
                let source_manager = source_manager_clone.clone();
                ws.on_upgrade(move |websocket| async move {
                    let (server, _) = source_manager.get_or_create_source(&video_id);
                    let client_id = Uuid::new_v4().to_string();
                    info!(
                        "New client {} connected to video source: {}",
                        client_id, video_id
                    );
                    server.handle_websocket(websocket, client_id).await;
                })
            });

    let port = env::var("SIGNALING_PORT")
        .unwrap_or_else(|_| "8080".to_string())
        .parse::<u16>()
        .map_err(|e| eyre::eyre!("Invalid SIGNALING_PORT: {}", e))?;

    let server_task = tokio::spawn(async move {
        info!("WebRTC signaling server listening on port {}", port);
        info!(
            "Connect to ws://localhost:{}/VIDEO_ID for specific video streams",
            port
        );
        warp::serve(websocket_route).run(([0, 0, 0, 0], port)).await;
    });

    let (_node, mut events) = DoraNode::init_from_env()?;

    while let Some(event) = events.next().await {
        match event {
            Event::Input { id, data, metadata } => {
                // Parse input ID to extract video_id
                // Expected format: "video_id/frame" or just "image" for backward compatibility
                let parts: Vec<&str> = id.as_str().split('/').collect();

                let (video_id, is_frame) = if parts.len() == 2 && parts[1] == "frame" {
                    (parts[0].to_string(), true)
                } else if id.as_str() == "image" {
                    // Backward compatibility: treat "image" as "default/frame"
                    ("default".to_string(), true)
                } else {
                    continue; // Skip non-frame inputs
                };

                if is_frame {
                    let encoding = if let Some(param) = metadata.parameters.get("encoding") {
                        match param {
                            dora_node_api::Parameter::String(s) => s.to_lowercase(),
                            _ => "rgb8".to_string(),
                        }
                    } else {
                        "rgb8".to_string()
                    };

                    if encoding == "rgb8" {
                        if let Some(data_arr) = data.as_any().downcast_ref::<UInt8Array>() {
                            let bytes = data_arr.values().to_vec();
                            let (_, frame_sender) = source_manager.get_or_create_source(&video_id);
                            let _ = frame_sender.send(bytes).await;
                        }
                    }
                }
            }
            Event::Stop(_) => {
                break;
            }
            _ => {}
        }
    }

    server_task.abort();
    Ok(())
}
