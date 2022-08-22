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
        pub unsafe extern "C" fn dora_init_operator(operator_context: *mut *mut std::ffi::c_void) -> isize {
            dora_operator_api::raw::dora_init_operator::<#operator_ty>(operator_context)
        }
    };

    let drop = quote! {
        #[no_mangle]
        pub unsafe extern "C" fn dora_drop_operator(operator_context: *mut std::ffi::c_void) {
            dora_operator_api::raw::dora_drop_operator::<#operator_ty>(operator_context)
        }
    };

    let on_input = quote! {
        #[no_mangle]
        pub unsafe extern "C" fn dora_on_input(
            metadata: *const std::ffi::c_void,
            data_start: *const u8,
            data_len: usize,
            output_fn_raw: dora_operator_api::raw::OutputFnRaw,
            output_context: *const std::ffi::c_void,
            operator_context: *mut std::ffi::c_void,
        ) -> isize {
            dora_operator_api::raw::dora_on_input::<#operator_ty>(
                metadata,
                data_start,
                data_len,
                output_fn_raw,
                output_context,
                operator_context,
            )
        }
    };

    Ok(quote! {
        #init
        #drop
        #on_input
    })
}
