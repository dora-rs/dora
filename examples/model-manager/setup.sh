#!/bin/bash

# Setup script for Model Manager
# This script installs all required dependencies

echo "========================================="
echo "   Model Manager Setup Script"
echo "========================================="
echo ""

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check Python installation
if ! command_exists python3; then
    echo "❌ Python 3 is not installed. Please install Python 3.7+ first."
    exit 1
fi

echo "✅ Python3 found: $(python3 --version)"

# Check pip installation
if ! command_exists pip3; then
    echo "❌ pip3 is not installed. Installing pip..."
    python3 -m ensurepip --default-pip
fi

echo "✅ pip3 found: $(pip3 --version)"
echo ""

# Ask user about ModelScope support
echo "Do you need ModelScope support for Chinese models (FunASR, etc.)?"
echo "This will install additional dependencies (~200MB)."
read -p "Install ModelScope? (y/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "📦 Installing full requirements (with ModelScope)..."
    pip3 install -r requirements-full.txt
    echo ""
    echo "✅ ModelScope support installed!"
else
    echo "📦 Installing core requirements (HuggingFace only)..."
    pip3 install -r requirements.txt
    echo ""
    echo "✅ Core dependencies installed!"
    echo "ℹ️  You can install ModelScope later with: pip install modelscope"
fi

echo ""
echo "========================================="
echo "   Setup Complete!"
echo "========================================="
echo ""
echo "You can now use the Model Manager:"
echo ""
echo "  # List cached models"
echo "  python3 download_models.py --list"
echo ""
echo "  # Download a model"
echo "  python3 download_models.py --download whisper-tiny"
echo ""
echo "  # Show help"
echo "  python3 download_models.py --help"
echo ""
echo "For more information, see README.md"
echo ""