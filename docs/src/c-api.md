# C API

## Operator

The operator API is a framework for you to implement. The implemented operator will be managed by `dora`. This framework enable us to make optimisation and provide advanced features.

The operator definition is composed of 3 functions, `dora_init_operator` that initialise the operator and its context. `dora_drop_operator` that free the memory, and `dora_on_event` that action the logic of the operator on receiving an input.

```c
{{#include ../../examples/c-dataflow/operator.c:0:29}}
```
### Try it out!

- Create an `operator.c` file:
```c
{{#include ../../examples/c-dataflow/operator.c}}
```

{{#include ../../examples/c-dataflow/README.md:40:46}}

- Link it in your graph as:
```yaml
{{#include ../../examples/c-dataflow/dataflow.yml:13:20}}
```

## Custom Node

The custom node API allow you to integrate `dora` into your application. It allows you to retrieve input and send output in any fashion you want. 

#### `init_dora_context_from_env`

`init_dora_context_from_env` initiate a node from environment variables set by `dora-coordinator` 

```c
void *dora_context = init_dora_context_from_env();
```

#### `dora_next_event`

`dora_next_event` waits for the next event (e.g. an input). Use `read_dora_event_type` to read the event's type. Inputs are of type `DoraEventType_Input`. To extract the ID and data of an input event, use `read_dora_input_id`  and `read_dora_input_data` on the returned pointer. It is safe to ignore any events and handle only the events that are relevant to the node.

```c
void *input = dora_next_input(dora_context);

// read out the ID as a UTF8-encoded string
char *id;
size_t id_len;
read_dora_input_id(input, &id, &id_len);

// read out the data as a byte array
char *data;
size_t data_len;
read_dora_input_data(input, &data, &data_len);
```

#### `dora_send_output`

`dora_send_output` send data from the node.

```c
char out_id[] = "tick";
char out_data[50];
dora_send_output(dora_context, out_id, strlen(out_id), out_data, out_data_len);
```
### Try it out!

- Create an `node.c` file:
```c
{{#include ../../examples/c-dataflow/node.c}}
```

{{#include ../../examples/c-dataflow/README.md:26:35}}

- Link it in your graph as:
```yaml
{{#include ../../examples/c-dataflow/dataflow.yml:6:12}}
```