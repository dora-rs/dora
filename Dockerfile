FROM erdosproject/pylot:v0.3.3

WORKDIR /home/erdos/workspace/pylot

RUN rm -rf /home/erdos/.rustup/toolchains/stable-x86_64-unknown-linux-gnu

RUN rustup update stable

RUN sudo apt-get update

RUN sudo apt-get install vim -y

WORKDIR /home/erdos/workspace/dora-rs

COPY src src

COPY Cargo.toml . 

RUN sudo chown erdos:erdos /home/erdos/workspace/dora-rs

RUN rustup default stable

RUN cargo build --release 

COPY examples examples


