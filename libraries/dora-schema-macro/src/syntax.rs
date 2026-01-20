use std::{collections::HashSet, mem};

use convert_case::{Case, Casing};
use quote::format_ident;
use syn::{
    Attribute, Ident, ItemTrait, Result,
    parse::{Parse, ParseStream},
};

#[derive(Debug)]
pub struct MethodDef {
    pub attrs: Vec<Attribute>,
    pub sig: syn::Signature,
    pub arguments: Vec<syn::PatType>,
    pub response: syn::Type,
}

impl MethodDef {
    pub fn variant_ident(&self) -> syn::Ident {
        let name =
            Casing::from_case(&self.sig.ident.to_string(), Case::Snake).to_case(Case::Pascal);
        format_ident!("{name}")
    }
}

#[derive(Debug)]
pub struct SchemaInput {
    pub item: ItemTrait,
    pub encoding: Option<Ident>,
    pub methods: Vec<MethodDef>,
}

impl Parse for SchemaInput {
    fn parse(input: ParseStream) -> Result<Self> {
        let attrs = input.call(Attribute::parse_outer)?;
        let mut item: ItemTrait = input.parse()?;
        item.attrs = attrs;
        let methods = mem::take(&mut item.items)
            .into_iter()
            .map(|m| {
                let syn::TraitItem::Fn(f) = m else {
                    return Err(syn::Error::new_spanned(m, "Expected method definition"));
                };
                let attrs = f.attrs;
                let sig = f.sig.clone();
                let arguments = f
                    .sig
                    .inputs
                    .iter()
                    .map(|arg| match arg {
                        syn::FnArg::Receiver(_) => Err(syn::Error::new_spanned(
                            arg,
                            "Methods cannot have self parameter",
                        )),
                        syn::FnArg::Typed(pat_type) => Ok(pat_type.clone()),
                    })
                    .collect::<Result<Vec<syn::PatType>>>()?;
                let response = match &f.sig.output {
                    syn::ReturnType::Type(_, ty) => (**ty).clone(),
                    syn::ReturnType::Default => syn::parse_quote!(()),
                };

                Ok(MethodDef {
                    attrs,
                    sig,
                    arguments,
                    response,
                })
            })
            .collect::<Result<Vec<MethodDef>>>()?;

        let mut seen = HashSet::new();
        seen.insert(format_ident!("handle"));
        for method in &methods {
            if !seen.insert(method.sig.ident.clone()) {
                return Err(syn::Error::new_spanned(
                    &method.sig.ident,
                    format!("Duplicate or reserved method name `{}`", method.sig.ident),
                ));
            }
        }

        Ok(SchemaInput {
            item,
            methods,
            encoding: None,
        })
    }
}

#[derive(Debug)]
pub struct AttributeArgs {
    pub encoding: Option<Ident>,
}

impl Parse for AttributeArgs {
    fn parse(input: ParseStream) -> Result<Self> {
        let mut encoding = None;

        while !input.is_empty() {
            let lookahead = input.lookahead1();
            if lookahead.peek(syn::Ident) {
                let ident: Ident = input.parse()?;
                if ident == "encoding" {
                    let _eq_token: syn::Token![=] = input.parse()?;
                    let value: Ident = input.parse()?;
                    encoding = Some(value);
                } else {
                    return Err(syn::Error::new_spanned(
                        ident.clone(),
                        format!("Unknown attribute `{}`", ident),
                    ));
                }
            } else {
                return Err(lookahead.error());
            }

            if input.peek(syn::Token![,]) {
                let _comma: syn::Token![,] = input.parse()?;
            }
        }

        Ok(AttributeArgs { encoding })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quote::quote;
    use syn::{parse_quote, parse2};

    #[test]
    fn parse_schema_input_valid_two_methods() {
        let input = quote! {
            trait Handler {
                fn list_nodes(req: ListNodesRequest) -> ListNodesResponse;
                fn get_node(req2: GetNodeRequest) -> GetNodeResponse;
            }
        };

        let schema: SchemaInput = parse2(input).expect("Failed to parse schema input");

        assert_eq!(schema.methods.len(), 2);

        assert_eq!(schema.methods[0].sig.ident, "list_nodes");
        assert_eq!(schema.methods[0].response, parse_quote!(ListNodesResponse));

        assert_eq!(schema.methods[1].sig.ident, "get_node");
        assert_eq!(schema.methods[1].response, parse_quote!(GetNodeResponse));
    }

    #[test]
    fn parse_schema_input_empty_return() {
        let input = quote! {
            trait Handler {
                fn list_nodes(req: ListNodesRequest);
            }
        };
        let _ = parse2::<SchemaInput>(input).expect("Failed to parse schema input");
    }

    #[test]
    fn parse_schema_input_duplicate_method_error() {
        let input = quote! {
            trait Handler {
                fn list_nodes(req: ListNodesRequest) -> ListNodesResponse;
                fn list_nodes(req: GetNodeRequest) -> GetNodeResponse;
            }
        };
        let _ = parse2::<SchemaInput>(input).expect_err("Expected parsing to fail");
    }

    #[test]
    fn parse_schema_input_reserved_name() {
        let input = quote! {
            trait Handler {
                fn handle(req: ServeRequest);
            }
        };
        let _ = parse2::<SchemaInput>(input).expect_err("Expected parsing to fail");
    }
}
