# Use a specific version of Python for consistency
FROM python:3.11

# Set non-interactive mode to prevent issues during installation
ENV DEBIAN_FRONTEND=noninteractive

# Set the working directory
WORKDIR /app

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN pip install --upgrade pip

# Install Rust using rustup
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y \
    && echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> /root/.bashrc

# Ensure Rust is available in all layers
ENV PATH="/root/.cargo/bin:$PATH"

# Install Dora CLI
RUN pip install dora-rs-cli

# Install uv package manager
RUN pip install uv

# Create a virtual environment
RUN uv venv --seed -p 3.11

# Install dependencies within the virtual environment
RUN uv pip install --upgrade pip

# Copy project files
COPY . .

# Default command (better alternative)
CMD ["/bin/bash"]
