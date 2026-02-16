# Core library for adora

## Generating adora schema

```bash
cargo run -p adora-core generate_schemas
```

## VSCode YAML Dataflow Support

We can pass the JSON Schema to VSCode [`redhat.vscode-yaml`](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml) to enables features such as:

- Type validation
- Suggestions
- Documentation

### Getting started

1. Install [`redhat.vscode-yaml`](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)

2. Open User Settings(JSON) in VSCode within `ctrl`+ `shift` + `p` search bar.

3. Add the following:

```json
  "yaml.schemas": {
    "https://raw.githubusercontent.com/adora-rs/adora/main/libraries/core/adora-schema.json": "/*"
  },
```

And you should be set! 🔥
