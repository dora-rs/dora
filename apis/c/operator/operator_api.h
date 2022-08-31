#ifndef __RUST_DORA_OPERATOR_API_C_WRAPPER__
#define __RUST_DORA_OPERATOR_API_C_WRAPPER__
#ifdef __cplusplus
extern "C"
{
#endif

#include <stddef.h>
#include "operator_types.h"

    void __dora_type_assertions()
    {
        DoraInitOperator_t __dora_init_operator = {.init_operator = dora_init_operator};
        DoraDropOperator_t __dora_drop_operator = {.drop_operator = dora_drop_operator};
        DoraOnInput_t __dora_on_input = {.on_input = dora_on_input};
    }

    DoraInitResult_t dora_init_operator(void);

    DoraResult_t dora_drop_operator(void *operator_context);

    OnInputResult_t dora_on_input(
        const Input_t *input,
        SendOutput_t send_output,
        void *operator_context);

#ifdef __cplusplus
} /* extern \"C\" */
#endif

#endif /* __RUST_DORA_OPERATOR_API_C_WRAPPER__ */
