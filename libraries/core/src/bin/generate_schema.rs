use std::{env, path::Path};

use dora_message::descriptor::Descriptor;
use schemars::schema_for;

fn main() {
    let schema = schema_for!(Descriptor);
    let raw_schema =
        serde_json::to_string_pretty(&schema).expect("Could not serialize schema to json");

    // Add additional properties to True, as #[derive(transparent)] of enums are not well handled.
    //
    // 'OneOf' such as Custom Nodes, Operators and Single Operators overwrite property values of the initial struct `Nodes`.`
    // which make the original properties such as `id` and `name` not validated by IDE extensions.
    let raw_schema = raw_schema.replace(
        "\"additionalProperties\": false",
        "\"additionalProperties\": true",
    );

    // Remove `serde(from=` nested field as they are not handled properly by `schemars`
    let raw_schema = raw_schema.replace(
        "\"python\": {
              \"$ref\": \"#/definitions/PythonSource\"
            }",
        "",
    );
    let raw_schema = raw_schema.replace(
        "{
            \"$ref\": \"#/definitions/Input\"
          }",
        "true",
    );

    // Get the Cargo root manifest directory
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");

    // Create the path for the new file next to Cargo.toml
    let new_file_path = Path::new(&manifest_dir).join("dora-schema.json");

    // write to file
    std::fs::write(new_file_path, raw_schema).expect("Could not write schema to file");
}
