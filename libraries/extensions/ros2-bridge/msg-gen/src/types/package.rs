use proc_macro2::Span;
use quote::{quote, ToTokens};
use syn::Ident;

use crate::types::{Action, Message, Service};

#[derive(Debug)]
pub struct Package {
    pub name: String,
    pub messages: Vec<Message>,
    pub services: Vec<Service>,
    pub actions: Vec<Action>,
}

impl Package {
    pub const fn new(name: String) -> Self {
        Self {
            name,
            messages: Vec::new(),
            services: Vec::new(),
            actions: Vec::new(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.messages.is_empty() && self.services.is_empty() && self.actions.is_empty()
    }

    pub fn message_structs(&self, gen_cxx_bridge: bool) -> (impl ToTokens, impl ToTokens) {
        if self.messages.is_empty() {
            // empty msg
            (quote! {}, quote! {})
        } else {
            let items = self
                .messages
                .iter()
                .map(|v| v.struct_token_stream(&self.name, gen_cxx_bridge));
            let defs = items.clone().map(|(def, _)| def);
            let impls = items.clone().map(|(_, im)| im);
            let def_tokens = quote! {
                #(#defs)*
            };
            let impl_tokens = quote! {
                #(#impls)*
            };
            (def_tokens, impl_tokens)
        }
    }

    fn message_aliases(&self, package_name: &Ident) -> impl ToTokens {
        if self.messages.is_empty() {
            quote! {
                // empty msg
            }
        } else {
            let items = self
                .messages
                .iter()
                .map(|v| v.alias_token_stream(package_name));
            quote! {
                pub mod msg {
                    #(#items)*
                }
            }
        }
    }

    fn service_aliases(&self, package_name: &Ident) -> impl ToTokens {
        if self.services.is_empty() {
            quote! {
                // empty msg
            }
        } else {
            let items = self
                .services
                .iter()
                .map(|v| v.alias_token_stream(package_name));
            quote! {
                pub mod service {
                    #(#items)*
                }
            }
        }
    }

    fn services_block(&self) -> impl ToTokens {
        if self.services.is_empty() {
            quote! {
                // empty srv
            }
        } else {
            let items = self.services.iter().map(|v| v.token_stream_with_mod());
            quote! {
                pub mod srv {
                    #(#items)*
                }  // srv
            }
        }
    }

    fn actions_block(&self) -> impl ToTokens {
        if self.actions.is_empty() {
            quote! {
                // empty srv
            }
        } else {
            let items = self.actions.iter().map(|v| v.token_stream_with_mod());
            quote! {
                pub mod action {
                    #(#items)*
                }  // action
            }
        }
    }

    pub fn aliases_token_stream(&self) -> impl ToTokens {
        let package_name = Ident::new(&self.name, Span::call_site());
        let aliases = self.message_aliases(&package_name);
        let service_aliases = self.service_aliases(&package_name);

        quote! {
            pub mod #package_name {
                #aliases
                #service_aliases
            }
        }
    }

    pub fn token_stream(&self, _gen_cxx_bridge: bool) -> impl ToTokens {
        let name = Ident::new(&self.name, Span::call_site());
        let services_block = self.services_block();
        let actions_block = self.actions_block();

        quote! {
            pub mod #name {
                #services_block
                #actions_block
            }
        }
    }
}
