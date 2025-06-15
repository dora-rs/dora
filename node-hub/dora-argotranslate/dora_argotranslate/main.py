"""TODO: Add docstring."""

import os

os.environ["ARGOS_DEVICE_TYPE"] = "auto"

import argostranslate.package
import argostranslate.translate
import pyarrow as pa
from dora import Node

from_code = os.getenv("SOURCE_LANGUAGE", "fr")
to_code = os.getenv("TARGET_LANGUAGE", "en")

# Download and install Argos Translate package
argostranslate.package.update_package_index()
available_packages = argostranslate.package.get_available_packages()
package_to_install = next(
    filter(
        lambda x: x.from_code == from_code and x.to_code == to_code,
        available_packages,
    ),
)
argostranslate.package.install_from_path(package_to_install.download())


def main():
    """TODO: Add docstring."""
    node = Node()
    while True:
        event = node.next()
        if event is None:
            break
        if event["type"] == "INPUT" and event["id"] == "text":
            text = event["value"][0].as_py()
            translated_text = argostranslate.translate.translate(
                text,
                from_code,
                to_code,
            )
            print(text, flush=True)
            print("translated: " + translated_text, flush=True)
            node.send_output(
                "text",
                pa.array([translated_text]),
                {"language": to_code},
            )
