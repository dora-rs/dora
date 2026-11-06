use std::{
    collections::{BTreeSet, HashMap, HashSet, VecDeque},
    path::PathBuf,
};

use proc_macro2::Span;
use quote::{ToTokens, format_ident, quote};
use syn::Ident;

use crate::types::{Action, Message, Service};

#[derive(Debug, Clone)]
pub struct Package {
    pub name: String,
    pub path: PathBuf,
    pub dependencies: Vec<Self>,
    pub messages: Vec<Message>,
    pub services: Vec<Service>,
    pub actions: Vec<Action>,
}

impl Package {
    /// Sorts messages topologically based on internal dependencies within this package.
    /// If message A references message B (including in fixed arrays, sequences, or nested types),
    /// message B will precede message A.
    pub fn sort_messages(&mut self) {
        let msg_names: HashSet<String> = self.messages.iter().map(|m| m.name.clone()).collect();

        let mut in_degrees: HashMap<String, usize> = HashMap::new();
        let mut dependents: HashMap<String, Vec<String>> = HashMap::new();

        for msg in &self.messages {
            let deps: BTreeSet<String> = msg
                .internal_dependencies()
                .into_iter()
                .filter(|dep| dep != &msg.name && msg_names.contains(dep))
                .collect();

            in_degrees.insert(msg.name.clone(), deps.len());
            for dep in deps {
                dependents.entry(dep).or_default().push(msg.name.clone());
            }
        }

        let mut ready: VecDeque<String> = self
            .messages
            .iter()
            .filter(|m| in_degrees.get(&m.name).copied().unwrap_or(0) == 0)
            .map(|m| m.name.clone())
            .collect();

        let mut sorted_names: Vec<String> = Vec::with_capacity(self.messages.len());
        let mut visited: HashSet<String> = HashSet::new();

        while let Some(name) = ready.pop_front() {
            if !visited.insert(name.clone()) {
                continue;
            }
            sorted_names.push(name.clone());

            if let Some(deps) = dependents.get(&name) {
                for dep in deps {
                    if let Some(deg) = in_degrees.get_mut(dep) {
                        *deg = deg.saturating_sub(1);
                        if *deg == 0 && !visited.contains(dep) {
                            ready.push_back(dep.clone());
                        }
                    }
                }
            }
        }

        if sorted_names.len() < self.messages.len() {
            for m in &self.messages {
                if visited.insert(m.name.clone()) {
                    sorted_names.push(m.name.clone());
                }
            }
        }

        let mut msg_map: HashMap<String, Message> = self
            .messages
            .drain(..)
            .map(|m| (m.name.clone(), m))
            .collect();

        self.messages = sorted_names
            .into_iter()
            .filter_map(|name| msg_map.remove(&name))
            .collect();
    }

    pub const fn new(name: String) -> Self {
        Self {
            name,
            path: PathBuf::new(),
            dependencies: Vec::new(),
            messages: Vec::new(),
            services: Vec::new(),
            actions: Vec::new(),
        }
    }

    pub const fn is_empty(&self) -> bool {
        self.messages.is_empty() && self.services.is_empty() && self.actions.is_empty()
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

    fn action_aliases(&self, package_name: &Ident) -> impl ToTokens {
        if self.actions.is_empty() {
            quote! {
                //empty msg
            }
        } else {
            let items = self
                .actions
                .iter()
                .map(|v| v.alias_token_stream(package_name));

            quote! {
                pub mod action {
                    #(#items)*
                }  // action
            }
        }
    }

    pub fn reuse_bindings_token_stream(&self) -> impl ToTokens {
        let mut messages = Vec::new();
        self.dependencies.iter().for_each(|dep| {
            let package_name = dep.name.as_str();
            let package_ident = format_ident!("{}", package_name);
            let package_header = format!("{}.rs.h", package_name);
            messages.push(quote! {
                include!(#package_header);
            });
            dep.messages.iter().for_each(|msg| {
                let message_name = format_ident!("{package_name}__{}", msg.name);
                let cxx_name = msg.name.as_str();
                messages.push(quote! {
                    #[namespace = #package_name]
                    #[cxx_name = #cxx_name]
                    type #message_name = crate::ros2::#package_ident::ffi::#message_name;
                });
            });
        });
        quote! {
            #(#messages)*
        }
    }

    pub fn dependencies_import_token_stream(&self) -> impl ToTokens {
        let imports = self.dependencies.iter().map(|dep| {
            let package_name = format_ident!("{}", dep.name);
            quote! {
                #[allow(unused_imports)]
                use crate::messages::#package_name::ffi::*;
            }
        });
        quote! {
            #(#imports)*
        }
    }

    pub fn aliases_token_stream(&self) -> impl ToTokens {
        let package_name = Ident::new(&self.name, Span::call_site());
        let aliases = self.message_aliases(&package_name);
        let service_aliases = self.service_aliases(&package_name);
        let action_aliases = self.action_aliases(&package_name);

        quote! {
            #aliases
            #service_aliases
            #action_aliases
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{
        Member,
        member::MemberType,
        message::Message,
        primitives::{NamedType, NestableType},
        sequences::Array,
    };

    #[test]
    fn sorts_messages_with_fixed_array_internal_dependency() {
        let mut pkg = Package::new("test_pkg".into());
        let outer = Message {
            package: "test_pkg".into(),
            name: "Outer".into(),
            members: vec![Member {
                r#type: MemberType::Array(Array {
                    value_type: NestableType::NamedType(NamedType("Inner".into())),
                    size: 4,
                }),
                name: "inners".into(),
                default: None,
            }],
            constants: vec![],
        };
        let inner = Message {
            package: "test_pkg".into(),
            name: "Inner".into(),
            members: vec![],
            constants: vec![],
        };

        // Insert Outer before Inner
        pkg.messages = vec![outer, inner];
        pkg.sort_messages();

        assert_eq!(pkg.messages[0].name, "Inner");
        assert_eq!(pkg.messages[1].name, "Outer");
    }
}
