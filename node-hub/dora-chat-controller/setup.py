from setuptools import setup, find_packages

setup(
    name="dora-chat-controller",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "dora-rs",
        "numpy",
        "pyarrow",
    ],
    entry_points={
        "console_scripts": [
            "dora-chat-controller=dora_chat_controller.main:main",
        ],
    },
    author="Dora Team",
    description="Chat controller node for voice chatbot - manages conversation flow",
    python_requires=">=3.7",
)