"""Setup script for dora-qwen3 package."""

from setuptools import setup, find_packages
import platform

# Read README for long description
with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

# Base requirements
install_requires = [
    "dora-rs>=0.3.9",
    "pyarrow>=10.0.0",
    "huggingface-hub>=0.20.0",
    "llama-cpp-python>=0.2.0",
]

# Add MLX dependencies for Apple Silicon
if platform.system() == "Darwin" and platform.machine() == "arm64":
    install_requires.extend([
        "mlx>=0.5.0",
        "mlx-lm>=0.10.0",
    ])

setup(
    name="dora-qwen3",
    version="0.1.0",
    author="Dora Team",
    author_email="dora@example.com",
    description="Qwen3-8B LLM node for Dora dataflows with MLX support",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/dora-rs/dora",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
    python_requires=">=3.9",
    install_requires=install_requires,
    entry_points={
        "console_scripts": [
            "dora-qwen3=dora_qwen3.main:main",
        ],
    },
    scripts=[],  # No additional scripts needed
)