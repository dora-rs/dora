import argostranslate.package
import argostranslate.translate
from dora import Node

node = Node()

from_code = "fr"
to_code = "en"

file_path = 'translated.txt'

def write_string_to_file(file_path, content):
    with open(file_path, 'w') as file:
        file.write(content)

# Download and install Argos Translate package
argostranslate.package.update_package_index()
available_packages = argostranslate.package.get_available_packages()
package_to_install = next(
    filter(
        lambda x: x.from_code == from_code and x.to_code == to_code, available_packages
    )
)
argostranslate.package.install_from_path(package_to_install.download())

# Translate


for event in node:
    if event["type"] == "INPUT":
        text = event["value"][0].as_py()
        translatedText = argostranslate.translate.translate(
            text,
            from_code,
            to_code,
        )
        write_string_to_file(file_path, translatedText)
        print(f"translated: {translatedText}", flush=True)
