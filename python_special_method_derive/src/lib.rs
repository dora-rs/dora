//! Derive macros to help with Python

extern crate proc_macro;
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, Data, DeriveInput, Fields};

// TODO: We should only list fields which are at least readable by Python users.
// This would require either reading the visibility modifier (easier) or checking
// the `pyo3` getter

/// Add a `__dir__` method to the struct in a `#[pymethods]` impl
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

/// Add a `__str__` and `__repr__` method to the struct in a `#[pymethods]` impl
#[proc_macro_derive(StrReprHelper)]
pub fn str_repr_helper_derive(input_stream: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input_stream as DeriveInput);

    // Get the name of the struct
    let name = &input.ident;

    let display_debug_derive_body = display_debug_derive(&input);

    let expanded = quote! {
        #display_debug_derive_body

        #[pyo3::pymethods]
        impl #name {
            pub fn __str__(&self) -> String {
                format!("{self}")
            }

            pub fn __repr__(&self) -> String {
                format!("{self:?}")
            }
        }
    };

    TokenStream::from(expanded)
}

macro_rules! create_body {
    ($input:expr, $ident:expr, $debug:expr) => {
        match &$input.data {
            syn::Data::Struct(s) => generate_fmt_impl_for_struct(s, $debug),
            syn::Data::Enum(e) => generate_fmt_impl_for_enum(e, $ident, $debug),
            syn::Data::Union(u) => {
                let error = syn::Error::new_spanned(u.union_token, "Unions are not supported");
                return proc_macro2::TokenStream::from(error.into_compile_error());
            }
        }
    };
}

// Internal function to generate Display and Debug impls.
// `Display` is used for `__str__`. `Debug` is used for `__repr__`.
fn display_debug_derive(input: &DeriveInput) -> proc_macro2::TokenStream {
    // Get the name of the struct
    let ident = &input.ident;

    let body_display = create_body!(input, ident, false);

    let body_debug = create_body!(input, ident, true);

    let expanded = quote! {
        impl std::fmt::Debug for #ident {
            fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(f, "{}(", stringify!(#ident))?;
                #(#body_debug)*
                write!(f, ")")
            }
        }

        impl std::fmt::Display for #ident {
            fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                write!(f, "{}(", stringify!(#ident))?;
                #(#body_display)*
                write!(f, ")")
            }
        }
    };

    expanded
}

fn generate_fmt_impl_for_struct(
    data_struct: &syn::DataStruct,
    debug: bool,
) -> Vec<proc_macro2::TokenStream> {
    let fields = &data_struct.fields;
    let n_fields = fields.len();
    let field_fmts = fields
        .iter()
        .enumerate()
        .map(|(i, field)| {
            // TODO: handle if debug. This may be checking visibility or only
            // displaying user specified idents. For now, repr the fields in Debug.
            let postfix = if i + 1 < n_fields { ", " } else { "" };
            match &field.ident {
                Some(ident) => {
                    if debug {
                        quote! {
                            write!(f, "{}: {:?}{}", stringify!(#ident), self.#ident, #postfix)?;
                        }
                    } else {
                        quote! {
                            write!(f, "{}: `{}`{}", stringify!(#ident), self.#ident, #postfix)?;
                        }
                    }
                }
                None => {
                    // If the field doesn't have a name, we generate a name based on its index
                    let index = syn::Index::from(i);
                    if debug {
                        quote! { write!(f, "{}: `{:?}`{}", stringify!(#index), self.#index, #postfix)?; }
                    } else {
                        quote! { write!(f, "{}: `{}`{}", stringify!(#index), self.#index, #postfix)?; }
                    }
                }
            }
        })
        .collect::<Vec<_>>();
    // Collect the mapped tokens into a TokenStream
    field_fmts
}

fn generate_fmt_impl_for_enum(
    data_enum: &syn::DataEnum,
    ident: &syn::Ident,
    _debug: bool,
) -> Vec<proc_macro2::TokenStream> {
    let n_variants = data_enum.variants.len();
    data_enum
        .variants
        .iter().enumerate()
        .map(|(i, variant)| {
            let variant_name = &variant.ident;
            // TODO: handle if debug. This may be only
            // displaying user specified variants.
            let variant_fmt = match &variant.fields {
                syn::Fields::Unit => {
                    // If the variant has no fields, we just print its name
                    quote! { write!(f, "{}", stringify!(#variant_name))?; }
                }
                syn::Fields::Named(fields) => {
                    // If the variant has named fields, we print each field's name and value
                    let field_fmts = fields.named.iter().map(|field| {
                        let field_name = field.ident.as_ref().unwrap();
                        let postfix = if i + 1 < n_variants { ", " } else { "" };
                        quote! {
                            write!(f, "{}: `{:?}`{}", stringify!(#field_name), self.#field_name, #postfix)?;
                        }
                    });
                    quote! {
                        write!(f, "{} {{ ", stringify!(#variant_name))?;
                        #( #field_fmts )*
                        write!(f, " }}")?;
                    }
                }
                syn::Fields::Unnamed(fields) => {
                    // If the variant has unnamed fields, we print each field's value without names
                    let field_fmts = fields.unnamed.iter().map(|field| {
                        quote! {
                            write!(f, "{:?}, ", self.#field)?;
                        }
                    });
                    quote! {
                        write!(f, "{}(", stringify!(#variant_name))?;
                        #( #field_fmts )*
                        write!(f, ")")?;
                    }
                }
            };
            quote! { #ident::#variant_name => { #variant_fmt } }
        })
        .collect::<Vec<_>>()
}
