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
#define EXPORT __attribute__((visibility("default")))
#endif

    EXPORT DoraInitResult_t dora_init_operator(void);

    EXPORT DoraResult_t dora_drop_operator(void *operator_context);

    EXPORT OnEventResult_t dora_on_event(
        RawEvent_t *event,
        const SendOutput_t *send_output,
        void *operator_context);

    static void __dora_type_assertions()
    {
        DoraInitOperator_t __dora_init_operator = {.init_operator = dora_init_operator};
        DoraDropOperator_t __dora_drop_operator = {.drop_operator = dora_drop_operator};
        DoraOnEvent_t __dora_on_event = {.on_event = dora_on_event};
    }
#ifdef __cplusplus
} /* extern \"C\" */
#endif

#endif /* __RUST_DORA_OPERATOR_API_C_WRAPPER__ */
