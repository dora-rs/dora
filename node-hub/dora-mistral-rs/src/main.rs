use dora_node_api::{dora_core::config::DataId, DoraNode, Event, IntoArrow};
use eyre::{Context, Report};
use mistralrs::{TextMessageRole, TextMessages, TextModelBuilder};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mistral_output = DataId::from("text".to_owned());
    let model_path = std::env::var("MODEL_PATH_OR_NAME")
        .unwrap_or_else(|_| "Qwen/Qwen2.5-0.5B-Instruct".to_owned());

    let model = TextModelBuilder::new(model_path)
        .with_logging()
        .build()
        .await
        .map_err(|e| Report::msg(format!("Model Build error: {}", e))) // Convert error
        .expect("Failed to build model");

    while let Some(event) = events.recv_async().await {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "text" => {
                    let received_string: &str =
                        TryFrom::try_from(&data).context("expected string message")?;

                    let messages = TextMessages::new()
                        // .add_message(
                        // TextMessageRole::System,
                        // "You are an AI agent with a specialty in robotics.",
                        // )
                        .add_message(TextMessageRole::User, received_string);

                    let response = model
                        .send_chat_request(messages)
                        .await
                        .map_err(|e| Report::msg(format!("Model Response error: {}", e))) // Convert error
                        .expect("Failed to get response from model");

                    let output = response.choices[0].message.content.as_ref().unwrap();

                    node.send_output(
                        mistral_output.clone(),
                        metadata.parameters,
                        output.into_arrow(),
                    )?;
                }
                other => eprintln!("Received input `{other}`"),
            },
            Event::Stop(_) => {
                println!("Received command");
            }
            Event::InputClosed { id } => {
                println!("input `{id}` was closed");
            }
            _ => {}
        }
    }

    Ok(())
}
