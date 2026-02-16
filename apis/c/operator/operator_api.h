#ifndef __RUST_ADORA_OPERATOR_API_C_WRAPPER__
#define __RUST_ADORA_OPERATOR_API_C_WRAPPER__
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

    EXPORT AdoraInitResult_t adora_init_operator(void);

    EXPORT AdoraResult_t adora_drop_operator(void *operator_context);

    EXPORT OnEventResult_t adora_on_event(
        RawEvent_t *event,
        const SendOutput_t *send_output,
        void *operator_context);

    static void __adora_type_assertions()
    {
        AdoraInitOperator_t __adora_init_operator = {.init_operator = adora_init_operator};
        AdoraDropOperator_t __adora_drop_operator = {.drop_operator = adora_drop_operator};
        AdoraOnEvent_t __adora_on_event = {.on_event = adora_on_event};
    }
#ifdef __cplusplus
} /* extern \"C\" */
#endif

#endif /* __RUST_ADORA_OPERATOR_API_C_WRAPPER__ */
