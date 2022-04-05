FROM erdosproject/pylot:v0.3.3

WORKDIR /home/erdos/workspace/pylot

RUN rm -rf /home/erdos/.rustup/toolchains/stable-x86_64-unknown-linux-gnu

RUN rustup update stable

COPY . /home/erdos/workspace/dora-rs

RUN sudo chown erdos /home/erdos/workspace/dora-rs

 