use zenoh::config::Config;

async fn register() {
    env_logger::init();
    let config = Config::default();
    let session = zenoh::open(config).await.unwrap();
    let mut subscriber = session.subscribe("a").await.unwrap();
}
