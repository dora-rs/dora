use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;

extern crate proc_macro;

#[proc_macro]
pub fn register_operator(item: TokenStream) -> TokenStream {
    // convert from `TokenStream` to `TokenStream2`, which is used by the
    // `syn` crate
    let item = TokenStream2::from(item);
    // generate the adora wrapper functions
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
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn adora_init_operator() -> adora_operator_api::types::AdoraInitResult {
            adora_operator_api::raw::adora_init_operator::<#operator_ty>()
        }

        const _ADORA_INIT_OPERATOR: adora_operator_api::types::AdoraInitOperator = adora_operator_api::types::AdoraInitOperator {
            init_operator: adora_init_operator,
        };
    };

    let drop = quote! {
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn adora_drop_operator(operator_context: *mut std::ffi::c_void)
            -> adora_operator_api::types::AdoraResult
        {
            adora_operator_api::raw::adora_drop_operator::<#operator_ty>(operator_context)
        }

        const _ADORA_DROP_OPERATOR: adora_operator_api::types::AdoraDropOperator = adora_operator_api::types::AdoraDropOperator {
            drop_operator: adora_drop_operator,
        };
    };

    let on_event = quote! {
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn adora_on_event(
            event: &mut adora_operator_api::types::RawEvent,
            send_output: &adora_operator_api::types::SendOutput,
            operator_context: *mut std::ffi::c_void,
        ) -> adora_operator_api::types::OnEventResult {
            adora_operator_api::raw::adora_on_event::<#operator_ty>(
                event, send_output, operator_context
            )
        }

        const _ADORA_ON_EVENT: adora_operator_api::types::AdoraOnEvent = adora_operator_api::types::AdoraOnEvent {
            on_event: adora_operator_api::types::OnEventFn(adora_on_event),
        };
    };

    // On Windows, we need to explicitly export symbols when linking a Rust staticlib
    // into a C++ DLL. This embeds linker directives directly into the object file.
    // SAFETY: The `.drectve` section is specifically designed for passing linker
    // directives on Windows. The content is a valid null-terminated string of
    // MSVC linker options that only affects symbol visibility.
    let windows_exports = quote! {
        #[cfg(target_os = "windows")]
        #[unsafe(link_section = ".drectve")]
        #[used]
        static _ADORA_EXPORTS: [u8; 77] =
            *b" /EXPORT:adora_init_operator /EXPORT:adora_drop_operator /EXPORT:adora_on_event ";
    };

    Ok(quote! {
        #init
        #drop
        #on_event
        #windows_exports
    })
}
