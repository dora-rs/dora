# C API

## Operator

The operator API gives you a framework for operator that is going to be managed by `dora`. This framework enable us to make optimisation and provide advanced features.

### Try it out!

- Create an `operator.c` file:
```c
{{#include ../../examples/c-operator/operator.c}}
```

- Copy `operator.h` header file:

```bash
cp apis/c/operator/api.h .
```

- And compile your C operator:
```bash
clang -c operator.c
clang -shared -v operator.o -o operator.so -fPIC
```

- Link it in your graph as:
```yaml
{{#include ../../binaries/coordinator/examples/mini-dataflow.yml:47:52}}
```