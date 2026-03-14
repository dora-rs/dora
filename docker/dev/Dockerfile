FROM python:3.12-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    curl \
    build-essential \
    pkg-config \
    libssl-dev \
    ffmpeg \
    libsm6 \
    libxext6 \
    clang \
    cmake \
    wget \
    gnupg \
    zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Apache Arrow from upstream repo
RUN wget -qO- https://apache.jfrog.io/artifactory/arrow/debian/apache-arrow-apt-source-latest-bookworm.deb -O /tmp/arrow.deb \
    && apt-get install -y /tmp/arrow.deb \
    && apt-get update \
    && apt-get install -y libarrow-dev libarrow-glib-dev \
    && rm -rf /var/lib/apt/lists/* /tmp/arrow.deb \
    || echo "Arrow dev packages not available, skipping (Arrow examples will not build)"

# Install Rust (required for Dora)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install uv
RUN pip install --no-cache-dir uv

# Install latest Dora
RUN pip install --no-cache-dir dora-rs-cli

# Create a working directory
WORKDIR /app

# Set environment variables
ENV PYTHONUNBUFFERED=1

# Default command when container starts
CMD ["bash"]
