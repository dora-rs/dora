use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;

extern crate proc_macro;

#[proc_macro]
pub fn register_operator(item: TokenStream) -> TokenStream {
    // convert from `TokenStream` to `TokenStream2`, which is used by the
    // `syn` crate
    let item = TokenStream2::from(item);
    // generate the dora wrapper functions
    let generated = register_operator_impl(&item).unwrap_or_else(|err| err.to_compile_error());
    // output the generated functions
    let tokens = quote! {
        #generated
    };
    // convert the type back from `TokenStream2` to `TokenStream`
    tokens.into()
}

/// Generates the wrapper functions for the annotated function.
fn register_operator_impl(item: &TokenStream2) -> syn::Result<TokenStream2> {
    // parse the type given to the `register_operator` macro
    let operator_ty: syn::TypePath = syn::parse2(item.clone())
        .map_err(|e| syn::Error::new(e.span(), "expected type as argument"))?;

    let init = quote! {
        #[no_mangle]
        pub unsafe extern "C" fn dora_init_operator() -> dora_operator_api::types::DoraInitResult {
            dora_operator_api::raw::dora_init_operator::<#operator_ty>()
        }

        const _DORA_INIT_OPERATOR: dora_operator_api::types::DoraInitOperator = dora_operator_api::types::DoraInitOperator {
            init_operator: dora_init_operator,
        };
    };

    let drop = quote! {
        #[no_mangle]
        pub unsafe extern "C" fn dora_drop_operator(operator_context: *mut std::ffi::c_void)
            -> dora_operator_api::types::DoraResult
        {
            dora_operator_api::raw::dora_drop_operator::<#operator_ty>(operator_context)
        }

        const _DORA_DROP_OPERATOR: dora_operator_api::types::DoraDropOperator = dora_operator_api::types::DoraDropOperator {
            drop_operator: dora_drop_operator,
        };
    };

    let on_event = quote! {
        #[no_mangle]
        pub unsafe extern "C" fn dora_on_event(
            event: &dora_operator_api::types::RawEvent,
            send_output: &dora_operator_api::types::SendOutput,
            operator_context: *mut std::ffi::c_void,
        ) -> dora_operator_api::types::OnEventResult {
            dora_operator_api::raw::dora_on_event::<#operator_ty>(
                event, send_output, operator_context
            )
        }

        const _DORA_ON_EVENT: dora_operator_api::types::DoraOnEvent = dora_operator_api::types::DoraOnEvent {
            on_event: dora_operator_api::types::OnEventFn(dora_on_event),
        };
    };

    Ok(quote! {
        #init
        #drop
        #on_event
    })
}
