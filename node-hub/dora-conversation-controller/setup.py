from setuptools import setup, find_packages

setup(
    name="dora-conversation-controller",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "dora-rs",
        "numpy",
        "pyarrow",
    ],
    entry_points={
        "console_scripts": [
            "dora-conversation-controller=dora_conversation_controller.main:main",
        ],
    },
    author="Dora",
    description="Conversation flow controller with backpressure management for Dora",
    python_requires=">=3.7",
)