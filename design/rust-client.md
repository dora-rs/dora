# Rust Client Design

## Brainstorm

framework vs library

### Framework

- Runtime process
  - Talks with other runtime processes
  - Across machines
  - loop
    - listen for inputs
    - invoke corresponding operator(s)
    - collect and forward outputs
- Operators
  - Connected to runtime
    - Via TCP socket (can be a separate process)
      - Single connection with high level message format, or
      - Separate connection per input/output
    - Dynamically linked as shared library
      - Runtime invokes specific handler message directly with input(s)
      - Outputs either:
        - Return a collection as result
        - Call runtime function to send out result
  - Input aggregation (i.e. waiting until multiple inputs are available)
    - by runtime -> aggregation specified in config file
    - by operator -> custom handling possible

### Library

- All sources/operator/sinks are separate processes that link a runtime library
- "Orchestrator" process
  - reads config file
  - launches processes accordingly
  - passes node config
    - as argument
    - via env variable
    - including input and output names
- Runtime library provides (async) functions to
  - wait for one or multiple inputs
  - with timeouts
  - send out outputs

