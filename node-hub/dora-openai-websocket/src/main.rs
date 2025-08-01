// Copyright 2023 Divy Srivastava <dj.srivastava23@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use base64::engine::general_purpose;
use base64::Engine;
use dora_cli::command::Executable;
use dora_cli::command::Start;
use dora_node_api::arrow::array::AsArray;
use dora_node_api::arrow::datatypes::DataType;
use dora_node_api::dora_core::config::DataId;
use dora_node_api::dora_core::config::NodeId;
use dora_node_api::dora_core::topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT;
use dora_node_api::into_vec;
use dora_node_api::DoraNode;
use dora_node_api::IntoArrow;
use dora_node_api::MetadataParameters;
use fastwebsockets::upgrade;
use fastwebsockets::Frame;
use fastwebsockets::OpCode;
use fastwebsockets::Payload;
use fastwebsockets::WebSocketError;
use futures_concurrency::future::Race;
use futures_util::future;
use futures_util::future::Either;
use futures_util::FutureExt;
use http_body_util::Empty;
use hyper::body::Bytes;
use hyper::body::Incoming;
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper::Request;
use hyper::Response;
use rand::random;
use serde;
use serde::Deserialize;
use serde::Serialize;
use std::fs;
use std::io::{self, Write};
use std::net::IpAddr;
use std::net::Ipv4Addr;
use tokio::net::TcpListener;
#[derive(Serialize, Deserialize, Debug)]
pub struct ErrorDetails {
    pub code: Option<String>,
    pub message: String,
    pub param: Option<String>,
    #[serde(rename = "type")]
    pub error_type: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum OpenAIRealtimeMessage {
    #[serde(rename = "session.update")]
    SessionUpdate { session: SessionConfig },
    #[serde(rename = "input_audio_buffer.append")]
    InputAudioBufferAppend {
        audio: String, // base64 encoded audio
    },
    #[serde(rename = "input_audio_buffer.commit")]
    InputAudioBufferCommit,
    #[serde(rename = "response.create")]
    ResponseCreate { response: ResponseConfig },
    #[serde(rename = "conversation.item.create")]
    ConversationItemCreate { item: ConversationItem },
    #[serde(rename = "conversation.item.truncate")]
    ConversationItemTruncate {
        item_id: String,
        content_index: u32,
        audio_end_ms: u32,
        #[serde(skip_serializing_if = "Option::is_none")]
        event_id: Option<String>,
    },
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SessionConfig {
    pub modalities: Vec<String>,
    pub instructions: String,
    pub voice: String,
    pub input_audio_format: String,
    pub output_audio_format: String,
    pub input_audio_transcription: Option<TranscriptionConfig>,
    pub turn_detection: Option<TurnDetectionConfig>,
    pub tools: Vec<serde_json::Value>,
    pub tool_choice: String,
    pub temperature: f32,
    pub max_response_output_tokens: Option<u32>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TranscriptionConfig {
    pub model: String,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TurnDetectionConfig {
    #[serde(rename = "type")]
    pub detection_type: String,
    pub threshold: f32,
    pub prefix_padding_ms: u32,
    pub silence_duration_ms: u32,
    pub interrupt_response: bool,
    pub create_response: bool,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ResponseConfig {
    pub modalities: Vec<String>,
    pub instructions: Option<String>,
    pub voice: Option<String>,
    pub output_audio_format: Option<String>,
    pub tools: Option<Vec<serde_json::Value>>,
    pub tool_choice: Option<String>,
    pub temperature: Option<f32>,
    pub max_output_tokens: Option<u32>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct ConversationItem {
    pub id: Option<String>,
    #[serde(rename = "type")]
    pub item_type: String,
    pub status: Option<String>,
    pub role: String,
    pub content: Vec<ContentPart>,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum ContentPart {
    #[serde(rename = "input_text")]
    InputText { text: String },
    #[serde(rename = "input_audio")]
    InputAudio {
        audio: String,
        transcript: Option<String>,
    },
    #[serde(rename = "text")]
    Text { text: String },
    #[serde(rename = "audio")]
    Audio {
        audio: String,
        transcript: Option<String>,
    },
}

// Incoming message types from OpenAI
#[derive(Serialize, Deserialize, Debug)]
#[serde(tag = "type")]
pub enum OpenAIRealtimeResponse {
    #[serde(rename = "error")]
    Error { error: ErrorDetails },
    #[serde(rename = "session.created")]
    SessionCreated { session: serde_json::Value },
    #[serde(rename = "session.updated")]
    SessionUpdated { session: serde_json::Value },
    #[serde(rename = "conversation.item.created")]
    ConversationItemCreated { item: serde_json::Value },
    #[serde(rename = "conversation.item.truncated")]
    ConversationItemTruncated { item: serde_json::Value },
    #[serde(rename = "response.audio.delta")]
    ResponseAudioDelta {
        response_id: String,
        item_id: String,
        output_index: u32,
        content_index: u32,
        delta: String, // base64 encoded audio
    },
    #[serde(rename = "response.audio.done")]
    ResponseAudioDone {
        response_id: String,
        item_id: String,
        output_index: u32,
        content_index: u32,
    },
    #[serde(rename = "response.text.delta")]
    ResponseTextDelta {
        response_id: String,
        item_id: String,
        output_index: u32,
        content_index: u32,
        delta: String,
    },
    #[serde(rename = "response.audio_transcript.delta")]
    ResponseAudioTranscriptDelta {
        response_id: String,
        item_id: String,
        output_index: u32,
        content_index: u32,
        delta: String,
    },
    #[serde(rename = "response.done")]
    ResponseDone { response: serde_json::Value },
    #[serde(rename = "input_audio_buffer.speech_started")]
    InputAudioBufferSpeechStarted {
        audio_start_ms: u32,
        item_id: String,
    },
    #[serde(rename = "input_audio_buffer.speech_stopped")]
    InputAudioBufferSpeechStopped { audio_end_ms: u32, item_id: String },
    #[serde(other)]
    Other,
}

fn convert_pcm16_to_f32(bytes: &[u8]) -> Vec<f32> {
    let mut samples = Vec::with_capacity(bytes.len() / 2);

    for chunk in bytes.chunks_exact(2) {
        let pcm16_sample = i16::from_le_bytes([chunk[0], chunk[1]]);
        let f32_sample = pcm16_sample as f32 / 32767.0;
        samples.push(f32_sample);
    }

    samples
}

fn convert_f32_to_pcm16(samples: &[f32]) -> Vec<u8> {
    let mut pcm16_bytes = Vec::with_capacity(samples.len() * 2);

    for &sample in samples {
        // Clamp to [-1.0, 1.0] and convert to i16
        let clamped = sample.max(-1.0).min(1.0);
        let pcm16_sample = (clamped * 32767.0) as i16;
        pcm16_bytes.extend_from_slice(&pcm16_sample.to_le_bytes());
    }

    pcm16_bytes
}

/// Replaces a placeholder in a file and writes the result to an output file.
///
/// # Arguments
///
/// * `input_path` - Path to the input file with placeholder text.
/// * `placeholder` - The placeholder text to search for (e.g., "{{PLACEHOLDER}}").
/// * `replacement` - The text to replace the placeholder with.
/// * `output_path` - Path to write the modified content.
fn replace_placeholder_in_file(
    input_path: &str,
    placeholder: &str,
    replacement: &str,
    output_path: &str,
) -> io::Result<()> {
    // Read the file content into a string
    let content = fs::read_to_string(input_path)?;

    // Replace the placeholder
    let modified_content = content.replace(placeholder, replacement);

    // Write the modified content to the output file
    let mut file = fs::File::create(output_path)?;
    file.write_all(modified_content.as_bytes())?;

    Ok(())
}

async fn handle_client(fut: upgrade::UpgradeFut) -> Result<(), WebSocketError> {
    let mut ws = fastwebsockets::FragmentCollector::new(fut.await?);

    let frame = ws.read_frame().await?;
    if frame.opcode != OpCode::Text {
        return Err(WebSocketError::InvalidConnectionHeader);
    }
    let data: OpenAIRealtimeMessage = serde_json::from_slice(&frame.payload).unwrap();
    let OpenAIRealtimeMessage::SessionUpdate { session } = data else {
        return Err(WebSocketError::InvalidConnectionHeader);
    };

    let input_audio_transcription = session
        .input_audio_transcription
        .map_or("moyoyo-whisper".to_string(), |t| t.model);
    let id = random::<u16>();
    let node_id = format!("server-{id}");
    let dataflow = format!("{input_audio_transcription}-{}.yml", id);
    let template = format!("{input_audio_transcription}-template-metal.yml");
    println!("Filling template: {}", template);
    replace_placeholder_in_file(&template, "NODE_ID", &node_id, &dataflow).unwrap();
    // Copy configuration file but replace the node ID with "server-id"
    // Read the configuration file and replace the node ID with "server-id"
    dora_cli::command::Command::Start(Start {
        dataflow,
        name: Some(node_id.to_string()),
        coordinator_addr: IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)),
        coordinator_port: DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
        attach: false,
        detach: true,
        hot_reload: false,
        uv: true,
    })
    .execute()
    .unwrap();
    let (mut node, mut events) =
        DoraNode::init_from_node_id(NodeId::from(node_id.clone())).unwrap();
    let serialized_data = OpenAIRealtimeResponse::SessionCreated {
        session: serde_json::Value::Null,
    };

    let payload =
        Payload::Bytes(Bytes::from(serde_json::to_string(&serialized_data).unwrap()).into());
    let frame = Frame::text(payload);
    ws.write_frame(frame).await?;
    loop {
        let event_fut = events.recv_async().map(Either::Left);
        let frame_fut = ws.read_frame().map(Either::Right);
        let event_stream = (event_fut, frame_fut).race();
        let mut finished = false;
        let frame = match event_stream.await {
            future::Either::Left(Some(ev)) => {
                let frame = match ev {
                    dora_node_api::Event::Input {
                        id,
                        metadata: _,
                        data,
                    } => {
                        if data.data_type() == &DataType::Utf8 {
                            let data = data.as_string::<i32>();
                            let str = data.value(0);
                            let serialized_data =
                                OpenAIRealtimeResponse::ResponseAudioTranscriptDelta {
                                    response_id: "123".to_string(),
                                    item_id: "123".to_string(),
                                    output_index: 123,
                                    content_index: 123,
                                    delta: str.to_string(),
                                };

                            let frame = Frame::text(Payload::Bytes(
                                Bytes::from(serde_json::to_string(&serialized_data).unwrap())
                                    .into(),
                            ));
                            frame
                        } else if id.contains("audio") {
                            let data: Vec<f32> = into_vec(&data).unwrap();
                            let data = convert_f32_to_pcm16(&data);
                            let serialized_data = OpenAIRealtimeResponse::ResponseAudioDelta {
                                response_id: "123".to_string(),
                                item_id: "123".to_string(),
                                output_index: 123,
                                content_index: 123,
                                delta: general_purpose::STANDARD.encode(data),
                            };
                            finished = true;

                            let frame = Frame::text(Payload::Bytes(
                                Bytes::from(serde_json::to_string(&serialized_data).unwrap())
                                    .into(),
                            ));
                            frame
                        } else if id.contains("stop") {
                            let serialized_data =
                                OpenAIRealtimeResponse::InputAudioBufferSpeechStopped {
                                    audio_end_ms: 123,
                                    item_id: "123".to_string(),
                                };
                            finished = true;

                            let frame = Frame::text(Payload::Bytes(
                                Bytes::from(serde_json::to_string(&serialized_data).unwrap())
                                    .into(),
                            ));
                            frame
                        } else {
                            unimplemented!()
                        }
                    }
                    dora_node_api::Event::Error(_) => {
                        // println!("Error in input: {}", s);
                        continue;
                    }
                    _ => break,
                };
                Some(frame)
            }
            future::Either::Left(None) => break,
            future::Either::Right(Ok(frame)) => {
                match frame.opcode {
                    OpCode::Close => break,
                    OpCode::Text | OpCode::Binary => {
                        let data: OpenAIRealtimeMessage =
                            serde_json::from_slice(&frame.payload).unwrap();

                        match data {
                            OpenAIRealtimeMessage::InputAudioBufferAppend { audio } => {
                                // println!("Received audio data: {}", audio);
                                let f32_data = audio;
                                // Decode base64 encoded audio data
                                let f32_data = f32_data.trim();
                                if f32_data.is_empty() {
                                    continue;
                                }

                                if let Ok(f32_data) = general_purpose::STANDARD.decode(f32_data) {
                                    let f32_data = convert_pcm16_to_f32(&f32_data);
                                    // Downsample to 16 kHz from 24 kHz
                                    let f32_data = f32_data
                                        .into_iter()
                                        .enumerate()
                                        .filter(|(i, _)| i % 3 != 0)
                                        .map(|(_, v)| v)
                                        .collect::<Vec<f32>>();

                                    let mut parameter = MetadataParameters::default();
                                    parameter.insert(
                                        "sample_rate".to_string(),
                                        dora_node_api::Parameter::Integer(16000),
                                    );
                                    node.send_output(
                                        DataId::from("audio".to_string()),
                                        parameter,
                                        f32_data.into_arrow(),
                                    )
                                    .unwrap();
                                }
                            }
                            OpenAIRealtimeMessage::InputAudioBufferCommit => break,
                            OpenAIRealtimeMessage::ResponseCreate { response } => {
                                if let Some(text) = response.instructions {
                                    node.send_output(
                                        DataId::from("text".to_string()),
                                        Default::default(),
                                        text.into_arrow(),
                                    )
                                    .unwrap();
                                }
                            }
                            _ => {}
                        }
                    }
                    _ => break,
                }
                None
            }
            future::Either::Right(Err(_)) => break,
        };
        if let Some(frame) = frame {
            ws.write_frame(frame).await?;
        }
        if finished {
            let serialized_data = OpenAIRealtimeResponse::ResponseDone {
                response: serde_json::Value::Null,
            };

            let payload = Payload::Bytes(
                Bytes::from(serde_json::to_string(&serialized_data).unwrap()).into(),
            );
            println!("Sending response done: {:?}", serialized_data);
            let frame = Frame::text(payload);
            ws.write_frame(frame).await?;
        };
    }

    Ok(())
}
async fn server_upgrade(
    mut req: Request<Incoming>,
) -> Result<Response<Empty<Bytes>>, WebSocketError> {
    let (response, fut) = upgrade::upgrade(&mut req)?;

    tokio::task::spawn(async move {
        if let Err(e) = tokio::task::unconstrained(handle_client(fut)).await {
            eprintln!("Error in websocket connection: {}", e);
        }
    });

    Ok(response)
}

fn main() -> Result<(), WebSocketError> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_io()
        .enable_time()
        .build()
        .unwrap();

    rt.block_on(async move {
        let listener = TcpListener::bind("127.0.0.1:8123").await?;
        println!("Server started, listening on {}", "127.0.0.1:8123");
        loop {
            let (stream, _) = listener.accept().await?;
            println!("Client connected");
            tokio::spawn(async move {
                let io = hyper_util::rt::TokioIo::new(stream);
                let conn_fut = http1::Builder::new()
                    .serve_connection(io, service_fn(server_upgrade))
                    .with_upgrades();
                if let Err(e) = conn_fut.await {
                    println!("An error occurred: {:?}", e);
                }
            });
        }
    })
}
