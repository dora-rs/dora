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
    /* Backward-compatible aliases for dora-hub C operators. */
    /* operator_types.h is auto-generated; aliases live here instead. */
    #define dora_init_operator          adora_init_operator
    #define dora_drop_operator          adora_drop_operator
    #define dora_on_event               adora_on_event
    #define dora_free_data              adora_free_data
    #define dora_free_input_id          adora_free_input_id
    #define dora_read_data              adora_read_data
    #define dora_read_input_id          adora_read_input_id
    #define dora_send_operator_output   adora_send_operator_output

    typedef AdoraResult_t              DoraResult_t;
    typedef AdoraInitResult_t          DoraInitResult_t;
    typedef AdoraDropOperator_t        DoraDropOperator_t;
    typedef AdoraInitOperator_t        DoraInitOperator_t;
    typedef AdoraOnEvent_t             DoraOnEvent_t;

    #define DORA_STATUS_CONTINUE        ADORA_STATUS_CONTINUE
    #define DORA_STATUS_STOP            ADORA_STATUS_STOP
    #define DORA_STATUS_STOP_ALL        ADORA_STATUS_STOP_ALL
    typedef AdoraStatus_t              DoraStatus_t;

#ifdef __cplusplus
} /* extern \"C\" */
#endif

#endif /* __RUST_ADORA_OPERATOR_API_C_WRAPPER__ */
