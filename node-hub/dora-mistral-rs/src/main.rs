use dora_node_api::{dora_core::config::DataId, DoraNode, Event, IntoArrow};
use eyre::{Context, Report};
use mistralrs::{
    IsqType, PagedAttentionMetaBuilder, TextMessageRole, TextMessages, TextModelBuilder,
};

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mistral_output = DataId::from("mistralrs".to_owned());

    let model = TextModelBuilder::new("deepseek-ai/DeepSeek-R1")
        .with_isq(IsqType::Q4K)
        .with_logging()
        .with_paged_attn(|| PagedAttentionMetaBuilder::default().build())
        .map_err(|e| Report::msg(format!("Paged attention meta error: {}", e))) // Convert error
        .expect("Failed to build model")
        .build()
        .await
        .map_err(|e| Report::msg(format!("Model Build error: {}", e))) // Convert error
        .expect("Failed to build model");

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => match id.as_str() {
                "message" => {
                    let received_string: &str =
                        TryFrom::try_from(&data).context("expected string message")?;
                    eprintln!("Received message: {}", received_string);

                    let messages = TextMessages::new()
                        .add_message(
                            TextMessageRole::System,
                            "You are an AI agent with a specialty in robotics.",
                        )
                        .add_message(TextMessageRole::User, received_string);

                    let response = model
                        .send_chat_request(messages)
                        .await
                        .map_err(|e| Report::msg(format!("Model Response error: {}", e))) // Convert error
                        .expect("Failed to get response from model");

                    let output = response.choices[0].message.content.as_ref().unwrap();
                    println!("{}", output);

                    dbg!(
                        response.usage.avg_prompt_tok_per_sec,
                        response.usage.avg_compl_tok_per_sec
                    );

                    node.send_output(
                        mistral_output.clone(),
                        metadata.parameters,
                        output.into_arrow(),
                    )?;
                }
                other => eprintln!("Received input `{other}`"),
            },
            Event::Stop => {
                println!("Received manual stop")
            }
            Event::InputClosed { id } => {
                println!("input `{id}` was closed");
            }
            _ => {}
        }
    }

    Ok(())
}
