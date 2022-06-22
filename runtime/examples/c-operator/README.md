# C-operator Example

Build with these steps:

```bash
cp ../../../api/c/operator/api.h .
clang -c operator.c
clang -shared -v operator.o -o operator.so
``` 
