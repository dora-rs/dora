# C-operator Example

Build with these steps:

- `cp ../../../api/c/operator/api.h .`
- `clang -c operator.c`
- `clang -shared -v operator.o -o operator.so`
