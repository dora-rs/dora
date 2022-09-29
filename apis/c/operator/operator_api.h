#ifndef __RUST_DORA_OPERATOR_API_C_WRAPPER__
#define __RUST_DORA_OPERATOR_API_C_WRAPPER__
#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include "operator_types.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

    EXPORT DoraInitResult_t dora_init_operator(void);

    EXPORT DoraResult_t dora_drop_operator(void *operator_context);

    EXPORT OnInputResult_t dora_on_input(
        const Input_t *input,
        const PrepareOutput_t *send_output,
        void *operator_context);

    void __dora_type_assertions()
    {
        DoraInitOperator_t __dora_init_operator = {.init_operator = dora_init_operator};
        DoraDropOperator_t __dora_drop_operator = {.drop_operator = dora_drop_operator};
        DoraOnInput_t __dora_on_input = {.on_input = dora_on_input};
    }
#ifdef __cplusplus
} /* extern \"C\" */
#endif

#endif /* __RUST_DORA_OPERATOR_API_C_WRAPPER__ */
