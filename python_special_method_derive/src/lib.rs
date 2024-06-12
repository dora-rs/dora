//! Derive macros to help with Python

extern crate proc_macro;
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, Data, DeriveInput, Fields};

// TODO: We should only list fields which are at least readable by Python users.
// This would require either reading the visibility modifier (easier) or checking
// the `pyo3` getter

/// Add a `fields` method to the struct.
///
/// Because we cannot have multiple `#[pymethods]` impls, this macro
/// produces a function called `fields` which should be called from the
/// PyO3 impl.
#[proc_macro_derive(DirHelper)]
pub fn dir_helper_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);

    // Get the name of the struct
    let name = &input.ident;

    // Generate code to match the struct's fields
    let expanded = match input.data {
        Data::Struct(data) => {
            match data.fields {
                Fields::Named(fields) => {
                    // If the struct has named fields extract their names
                    let field_names = fields
                        .named
                        .iter()
                        .map(|f| f.ident.as_ref().unwrap())
                        .collect::<Vec<_>>();

                    // Prepare an array where the elements are expressions that prepare the field vec
                    let mut assigner = proc_macro2::TokenStream::new();
                    quote_into::quote_into!(assigner += [#{
                        for name in field_names {
                            quote_into::quote_into!(assigner += (names.push(stringify!(#name).to_string())),)
                        }
                    }];);
                    quote! {
                        #[pyo3::pymethods]
                        impl #name {
                            pub fn __dir__(&self) -> Vec<String> {
                                let mut names = Vec::new();
                                #assigner
                                names
                            }
                        }
                    }
                }
                Fields::Unit => {
                    // If the struct has no fields
                    quote! {
                        #[pyo3::pymethods]
                        impl #name {
                            pub fn __dir__(&self) -> Vec<String> {
                                Vec::new()
                            }
                        }
                    }
                }
                Fields::Unnamed(_) => {
                    quote! {
                        compile_error!("Unnamed fields for struct are not supported for DirHelper derive.");
                    }
                }
            }
        }
        Data::Enum(_) => {
            quote! {
                compile_error!("Enums are not supported for DirHelper derive");
            }
        }
        Data::Union(_) => {
            quote! {
                compile_error!("Unions are not supported for DirHelper derive");
            }
        }
    };
    TokenStream::from(expanded)
}
