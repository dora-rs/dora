# Dora-OpenAI-Realtime (ROOT Repo)

## Front End

### Build Client

```bash
git clone git@github.com:haixuanTao/moly.git --branch dora-backend-support
cd moly
cargo build --release
```

### Run Client

```bash
cd moly
cargo run -r
```

## Server

### Build server

```bash
uv venv --seed -p 3.11
dora build whisper-template-metal.yml --uv ## very long process
```

### Run server

```bash
source .venv/bin/activate
dora up
cargo run --release -p dora-openai-websocket
```

## On finish

```bash
dora destroy
```

## GUI

- Go to MolyServer Tab
- Add a custom Provider
- In API Host, use:

  - Name: dora-websocket
  - API Host: ws://127.0.0.1:8123
  - Type: OpenAI Realtime

- Then go to Chat Tab
- New Chat
- ( Make sure the servver is running with: `cargo run --release -p dora-openai-websocket`)
- On bottom right, click on 🎧 icon.
  > If nothing happen is that the server is not found.
- Click on start
- Wait for the first AI greeting
- Start speaking!
- You should get AI response!

### WIP: Moyoyo

## {Recommended} Install git-lfs

```bash
brew install git-lfs # MacOS
```

## Clone Moxin Voice Chat

```bash
git lfs install
git clone https://github.com/moxin-org/moxin-voice-chat.git
```
