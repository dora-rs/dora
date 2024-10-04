import argostranslate.package
import argostranslate.translate

from dora import Node

from_code = "fr"
to_code = "en"

# Download and install Argos Translate package
argostranslate.package.update_package_index()
available_packages = argostranslate.package.get_available_packages()
package_to_install = next(
    filter(
        lambda x: x.from_code == from_code and x.to_code == to_code, available_packages
    )
)
argostranslate.package.install_from_path(package_to_install.download())

def main():
    node = Node()
    while True:
        event = node.next()
        if event is None:
            break
        if event["type"] == "INPUT" and event["id"] == "text":
            text = event["value"][0].as_py()
            translatedText = argostranslate.translate.translate(
                text,
                from_code,
                to_code,
            )
            print(f"translated: {translatedText}", flush=True)