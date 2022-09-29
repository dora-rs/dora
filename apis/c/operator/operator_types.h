/*! \file */
/*******************************************
 *                                         *
 *  File auto-generated by `::safer_ffi`.  *
 *                                         *
 *  Do not manually edit this file.        *
 *                                         *
 *******************************************/

#ifndef __RUST_DORA_OPERATOR_API_C__
#define __RUST_DORA_OPERATOR_API_C__
#ifdef __cplusplus
extern "C" {
#endif


#include <stddef.h>
#include <stdint.h>

/** \brief
 *  Same as [`Vec<T>`][`rust::Vec`], but with guaranteed `#[repr(C)]` layout
 */
typedef struct Vec_uint8 {
    /** <No documentation available> */
    uint8_t * ptr;

    /** <No documentation available> */
    size_t len;

    /** <No documentation available> */
    size_t cap;
} Vec_uint8_t;

/** <No documentation available> */
typedef struct DoraResult {
    /** <No documentation available> */
    Vec_uint8_t error;
} DoraResult_t;

/** <No documentation available> */
typedef struct DoraDropOperator {
    /** <No documentation available> */
    DoraResult_t (*drop_operator)(void *);
} DoraDropOperator_t;

/** <No documentation available> */
typedef struct DoraInitResult {
    /** <No documentation available> */
    DoraResult_t result;

    /** <No documentation available> */
    void * operator_context;
} DoraInitResult_t;

/** <No documentation available> */
typedef struct DoraInitOperator {
    /** <No documentation available> */
    DoraInitResult_t (*init_operator)(void);
} DoraInitOperator_t;

/** <No documentation available> */
/** \remark Has the same ABI as `uint8_t` **/
#ifdef DOXYGEN
typedef
#endif
enum DoraStatus {
    /** <No documentation available> */
    DORA_STATUS_CONTINUE = 0,
    /** <No documentation available> */
    DORA_STATUS_STOP = 1,
}
#ifndef DOXYGEN
; typedef uint8_t
#endif
DoraStatus_t;

/** <No documentation available> */
typedef struct OnInputResult {
    /** <No documentation available> */
    DoraResult_t result;

    /** <No documentation available> */
    DoraStatus_t status;
} OnInputResult_t;

/** <No documentation available> */
typedef struct Metadata {
    /** <No documentation available> */
    Vec_uint8_t open_telemetry_context;
} Metadata_t;

/** <No documentation available> */
typedef struct Input {
    /** <No documentation available> */
    Vec_uint8_t id;

    /** <No documentation available> */
    Vec_uint8_t data;

    /** <No documentation available> */
    Metadata_t metadata;
} Input_t;

/** \brief
 *  Like [`slice_ref`] and [`slice_mut`], but with any lifetime attached
 *  whatsoever.
 *
 *  It is only intended to be used as the parameter of a **callback** that
 *  locally borrows it, due to limitations of the [`ReprC`][
 *  `trait@crate::layout::ReprC`] design _w.r.t._ higher-rank trait bounds.
 *
 *  # C layout (for some given type T)
 *
 *  ```c
 *  typedef struct {
 *  // Cannot be NULL
 *  T * ptr;
 *  size_t len;
 *  } slice_T;
 *  ```
 *
 *  # Nullable pointer?
 *
 *  If you want to support the above typedef, but where the `ptr` field is
 *  allowed to be `NULL` (with the contents of `len` then being undefined)
 *  use the `Option< slice_ptr<_> >` type.
 */
typedef struct slice_raw_uint8 {
    /** \brief
     *  Pointer to the first element (if any).
     */
    uint8_t * ptr;

    /** \brief
     *  Element count
     */
    size_t len;
} slice_raw_uint8_t;

/** \brief
 *  `Box<dyn 'static + Send + FnMut() -> Ret>`
 */
typedef struct BoxDynFnMut0_slice_raw_uint8 {
    /** <No documentation available> */
    void * env_ptr;

    /** <No documentation available> */
    slice_raw_uint8_t (*call)(void *);

    /** <No documentation available> */
    void (*free)(void *);
} BoxDynFnMut0_slice_raw_uint8_t;

/** \brief
 *  `Box<dyn 'static + Send + FnMut() -> Ret>`
 */
typedef struct BoxDynFnMut0_DoraResult {
    /** <No documentation available> */
    void * env_ptr;

    /** <No documentation available> */
    DoraResult_t (*call)(void *);

    /** <No documentation available> */
    void (*free)(void *);
} BoxDynFnMut0_DoraResult_t;

/** <No documentation available> */
typedef struct Output {
    /** <No documentation available> */
    BoxDynFnMut0_slice_raw_uint8_t data_mut;

    /** <No documentation available> */
    BoxDynFnMut0_DoraResult_t send;
} Output_t;

/** <No documentation available> */
typedef struct PrepareOutputResult {
    /** <No documentation available> */
    DoraResult_t result;

    /** <No documentation available> */
    Output_t output;
} PrepareOutputResult_t;

/** <No documentation available> */
typedef struct OutputMetadata {
    /** <No documentation available> */
    Vec_uint8_t id;

    /** <No documentation available> */
    Metadata_t metadata;

    /** <No documentation available> */
    size_t data_len;
} OutputMetadata_t;

/** \brief
 *  `Arc<dyn Send + Sync + Fn(A1) -> Ret>`
 */
typedef struct ArcDynFn1_PrepareOutputResult_OutputMetadata {
    /** <No documentation available> */
    void * env_ptr;

    /** <No documentation available> */
    PrepareOutputResult_t (*call)(void *, OutputMetadata_t);

    /** <No documentation available> */
    void (*release)(void *);

    /** <No documentation available> */
    void (*retain)(void *);
} ArcDynFn1_PrepareOutputResult_OutputMetadata_t;

/** <No documentation available> */
typedef struct PrepareOutput {
    /** <No documentation available> */
    ArcDynFn1_PrepareOutputResult_OutputMetadata_t prepare_output;
} PrepareOutput_t;

/** <No documentation available> */
typedef struct DoraOnInput {
    /** <No documentation available> */
    OnInputResult_t (*on_input)(Input_t const *, PrepareOutput_t const *, void *);
} DoraOnInput_t;


#ifdef __cplusplus
} /* extern \"C\" */
#endif

#endif /* __RUST_DORA_OPERATOR_API_C__ */
