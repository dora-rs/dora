# C API

## Operator

The operator API is a framework for you to implement. The implemented operator will be managed by `dora`. This framework enable us to make optimisation and provide advanced features.

The operator definition is composed of 3 functions, `dora_init_operator` that initialise the operator and its context. `dora_drop_operator` that free the memory, and `dora_on_input` that action the logic of the operator on receiving an input.

```c
int dora_init_operator(void **operator_context)
{
    // allocate a single byte to store a counter
    // (the operator context pointer can be used to keep arbitrary data between calls)
    void *context = malloc(1);

    char *context_char = (char *)context;
    *context_char = 0;

    *operator_context = context;

    return 0;
}

void dora_drop_operator(void *operator_context)
{
    free(operator_context);
}

int dora_on_input(
    const char *id_start,
    size_t id_len,
    const char *data_start,
    size_t data_len,
    const int (*output_fn_raw)(const char *id_start,
                               size_t id_len,
                               const char *data_start,
                               size_t data_len,
                               const void *output_context),
    void *output_context,
    const void *operator_context)
{
    // handle the input ...
    // (sending outputs is possible using `output_fn_raw`)
    // (the `operator_context` is the pointer created in `dora_init_operator`, i.e., a counter in our case)
}
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

#### `dora_next_input`

`dora_next_input` waits for the next input. To extract the input ID and data, use `read_dora_input_id`  and `read_dora_input_data` on the returned pointer.

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
char out_data[] = {0, 0, 0};
dora_send_output(dora_context, out_id, strlen(out_id), &out_data, sizeof out_data);
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