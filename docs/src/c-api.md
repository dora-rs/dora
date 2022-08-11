# C API

## Custom Node

The custom node API allow you to integrate `dora` into your application. It allows you to retrieve input and send output in any fashion you want. 

#### `init_dora_context_from_env`

`init_dora_context_from_env` initiate a node from environment variables set by `dora-coordinator` 

```c
void *dora_context = init_dora_context_from_env();
```

#### `dora_next_input` and `read_dora_input_data`

`dora_next_input` and `read_dora_input_data` gives you the next input received.

```c
void *input = dora_next_input(dora_context);

char *data;
size_t data_len;
read_dora_input_data(input, &data, &data_len);
```

#### `dora_send_output`

`dora_send_output` send data from the node.

```c
char out_id[] = "tick";
dora_send_output(dora_context, out_id, strlen(out_id), &i, 1);
```
### Try it out!

- Create an `node.c` file:
```c
{{#include ../../examples/c-dataflow/node.c}}
```

{{#include ../../examples/c-dataflow/README.md:26:35}}


## Operator

The operator API gives you a framework for operator that is going to be managed by `dora`. This framework enable us to make optimisation and provide advanced features.

The operator definition is composed of 3 functions, `dora_init_operator` that initialise the operator and its context. `dora_drop_operator` that free the memory, and `dora_on_input` that action the logic of the operator on receiving an input.

```c
int dora_init_operator(void **operator_context)
{
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
  ...
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
{{#include ../../binaries/coordinator/examples/mini-dataflow.yml:47:52}}
```