#include <stddef.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#endif
#ifdef __unix__
#define EXPORT
#endif

EXPORT int dora_init_operator(void **operator_context);

EXPORT void dora_drop_operator(void *operator_context);

EXPORT int dora_on_input(
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
    const void *operator_context);
