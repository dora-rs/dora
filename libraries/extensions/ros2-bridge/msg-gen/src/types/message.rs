use quote::{format_ident, quote, ToTokens};
use syn::Ident;

use super::{primitives::*, sequences::Array, ConstantType, MemberType};

/// A member of a structure
#[derive(Debug, Clone)]
pub struct Member {
    /// The name of the member
    pub name: String,
    /// The type of the member
    pub r#type: MemberType,
    /// The default value of the member (optional)
    pub default: Option<Vec<String>>,
}

impl Member {
    fn dummy() -> Self {
        Self {
            name: "structure_needs_at_least_one_member".into(),
            r#type: BasicType::U8.into(),
            default: None,
        }
    }

    fn name_token(&self) -> impl ToTokens {
        if RUST_KEYWORDS.contains(&self.name.as_str()) {
            format_ident!("{}_", self.name)
        } else {
            format_ident!("{}", self.name)
        }
    }

    fn rust_type_def(&self, package: &str) -> impl ToTokens {
        let name = self.name_token();
        let (attr, type_) = self.r#type.type_tokens(package);
        quote! { #attr pub #name: #type_, }
    }

    fn default_value(&self) -> impl ToTokens {
        let name = self.name_token();
        self.default.as_ref().map_or_else(
            || quote! { #name: crate::_core::InternalDefault::_default(), },
            |default| {
                let default = self.r#type.value_tokens(default);
                quote! { #name: #default, }
            },
        )
    }

    fn raw_type_def(&self, package: &str) -> impl ToTokens {
        let name = self.name_token();
        let type_ = self.r#type.raw_type_tokens(package);
        quote! { pub #name: #type_, }
    }

    fn ffi_to_rust(&self) -> impl ToTokens {
        let name = self.name_token();
        let value = match &self.r#type {
            MemberType::NestableType(NestableType::BasicType(_)) => quote! { self.#name },
            MemberType::Array(Array {
                value_type: NestableType::BasicType(_),
                ..
            }) => quote! { self.#name.clone() },
            _ => quote! { self.#name.to_rust() },
        };

        quote! { #name: #value, }
    }

    fn raw_ref_type_def(&self, package: &str) -> impl ToTokens {
        let name = self.name_token();
        let type_ = self.r#type.raw_ref_type_tokens(package);
        quote! { pub #name: #type_, }
    }

    fn ffi_from_rust(&self) -> impl ToTokens {
        let name = self.name_token();
        let value = match &self.r#type {
            MemberType::NestableType(NestableType::BasicType(_)) => quote! { from.#name },
            MemberType::Array(Array {
                value_type: NestableType::BasicType(_),
                ..
            }) => quote! { from.#name.clone() },
            _ => quote! { _FFIFromRust::from_rust(&from.#name) },
        };
        quote! { #name: #value, }
    }
}

/// A constant definition
#[derive(Debug, Clone)]
pub struct Constant {
    /// The name of the constant
    pub name: String,
    /// The type of the constant
    pub r#type: ConstantType,
    /// The value of the constant
    pub value: Vec<String>,
}

impl Constant {
    fn token_stream(&self) -> impl ToTokens {
        let name = format_ident!("{}", self.name);
        let type_ = self.r#type.type_tokens();
        let value = self.r#type.value_tokens(&self.value);
        quote! { pub const #name: #type_ = #value; }
    }

    fn cxx_method_def_token_stream(&self, struct_name: &str, package_name: &str) -> impl ToTokens {
        let name = format_ident!("const_{package_name}__{struct_name}_{}", self.name);
        let cxx_name = format_ident!("const_{struct_name}_{}", self.name);
        let type_ = self.r#type.type_tokens();
        quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_name]
            pub fn #name () -> #type_;
        }
    }

    fn cxx_method_impl_token_stream(&self, struct_raw_name: &Ident) -> impl ToTokens {
        let const_name = format_ident!("{}", self.name);
        let name = format_ident!("const_{struct_raw_name}_{}", self.name);
        let type_ = self.r#type.type_tokens();
        quote! {
            #[allow(non_snake_case, dead_code)]
            fn #name () -> #type_ { ffi::#struct_raw_name::#const_name }
        }
    }
}

/// A message definition
#[derive(Debug, Clone)]
pub struct Message {
    /// The package name
    pub package: String,
    /// The name of the message
    pub name: String,
    /// The list of the members
    pub members: Vec<Member>,
    /// The list of the constants
    pub constants: Vec<Constant>,
}

impl Message {
    pub fn struct_token_stream(
        &self,
        package_name: &str,
        gen_cxx_bridge: bool,
    ) -> (impl ToTokens, impl ToTokens) {
        let cxx_name = format_ident!("{}", self.name);
        let struct_raw_name = format_ident!("{package_name}__{}", self.name);

        let rust_type_def_inner = self.members.iter().map(|m| m.rust_type_def(&self.package));
        let constants_def_inner = self.constants.iter().map(|c| c.token_stream());
        let cxx_const_def_inner = self
            .constants
            .iter()
            .map(|c| c.cxx_method_def_token_stream(&self.name, package_name));
        let cxx_const_impl_inner = self
            .constants
            .iter()
            .map(|c| c.cxx_method_impl_token_stream(&struct_raw_name));
        let rust_type_default_inner = self.members.iter().map(|m| m.default_value());

        let (attributes, cxx_consts) = if gen_cxx_bridge {
            let attributes = quote! {
                #[namespace = #package_name]
                #[cxx_name = #cxx_name]
            };
            let consts = quote! {
                extern "Rust" {
                    #(#cxx_const_def_inner)*
                }
            };
            (attributes, consts)
        } else {
            (quote! {}, quote! {})
        };

        let def = if self.members.is_empty() {
            quote! {
                #[allow(non_camel_case_types)]
                #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
                #attributes
                pub struct #struct_raw_name {
                    #[serde(skip)]
                    pub(super) _dummy: u8,
                }

                #cxx_consts
            }
        } else {
            quote! {
                #[allow(non_camel_case_types)]
                #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
                #attributes
                pub struct #struct_raw_name {
                    #(#rust_type_def_inner)*
                }

                #cxx_consts
            }
        };
        let default = if self.members.is_empty() {
            quote! {
                Self {
                    _dummy: 0,
                }
            }
        } else {
            quote! {
                Self {
                    #(#rust_type_default_inner)*
                }
            }
        };
        let impls = quote! {
            impl ffi::#struct_raw_name {
                #(#constants_def_inner)*

            }

            impl crate::_core::InternalDefault for ffi::#struct_raw_name {
                fn _default() -> Self {
                    #default
                }
            }

            impl std::default::Default for ffi::#struct_raw_name {
                #[inline]
                fn default() -> Self {
                    crate::_core::InternalDefault::_default()
                }
            }

            impl crate::ros2_client::Message for ffi::#struct_raw_name {}

            #(#cxx_const_impl_inner)*
        };

        (def, impls)
    }

    pub fn topic_def(&self, package_name: &str) -> (impl ToTokens, impl ToTokens) {
        if self.members.is_empty() {
            return (quote! {}, quote! {});
        };

        let topic_name = format_ident!("Topic__{package_name}__{}", self.name);
        let cxx_topic_name = format_ident!("Topic_{}", self.name);
        let create_topic = format_ident!("new__Topic__{package_name}__{}", self.name);
        let cxx_create_topic = format!("create_topic_{package_name}_{}", self.name);

        let publisher_name = format_ident!("Publisher__{package_name}__{}", self.name);
        let cxx_publisher_name = format_ident!("Publisher_{}", self.name);
        let create_publisher = format_ident!("new__Publisher__{package_name}__{}", self.name);
        let cxx_create_publisher = format_ident!("create_publisher");

        let struct_raw_name = format_ident!("{package_name}__{}", self.name);
        let struct_raw_name_str = struct_raw_name.to_string();
        let self_name = &self.name;

        let publish = format_ident!("publish__{package_name}__{}", self.name);
        let cxx_publish = format_ident!("publish");

        let subscription_name = format_ident!("Subscription__{package_name}__{}", self.name);
        let subscription_name_str = subscription_name.to_string();
        let cxx_subscription_name = format_ident!("Subscription_{}", self.name);
        let create_subscription = format_ident!("new__Subscription__{package_name}__{}", self.name);
        let cxx_create_subscription = format_ident!("create_subscription");

        let matches = format_ident!("matches__{package_name}__{}", self.name);
        let cxx_matches = format_ident!("matches");
        let downcast = format_ident!("downcast__{package_name}__{}", self.name);
        let cxx_downcast = format_ident!("downcast");

        let def = quote! {
            #[namespace = #package_name]
            #[cxx_name = #cxx_topic_name]
            type #topic_name;
            #[cxx_name = #cxx_create_topic]
            fn #create_topic(self: &Ros2Node, name_space: &str, base_name: &str, qos: Ros2QosPolicies) -> Result<Box<#topic_name>>;
            #[cxx_name = #cxx_create_publisher]
            fn #create_publisher(self: &mut Ros2Node, topic: &Box<#topic_name>, qos: Ros2QosPolicies) -> Result<Box<#publisher_name>>;
            #[cxx_name = #cxx_create_subscription]
            fn #create_subscription(self: &mut Ros2Node, topic: &Box<#topic_name>, qos: Ros2QosPolicies, events: &mut CombinedEvents) -> Result<Box<#subscription_name>>;

            #[namespace = #package_name]
            #[cxx_name = #cxx_publisher_name]
            type #publisher_name;
            #[namespace = #package_name]
            #[cxx_name = #cxx_publish]
            fn #publish(self: &mut #publisher_name, message: #struct_raw_name) -> Result<()>;

            #[namespace = #package_name]
            #[cxx_name = #cxx_subscription_name]
            type #subscription_name;

            #[namespace = #package_name]
            #[cxx_name = #cxx_matches]
            fn #matches(self: &#subscription_name, event: &CombinedEvent) -> bool;
            #[namespace = #package_name]
            #[cxx_name = #cxx_downcast]
            fn #downcast(self: &#subscription_name, event: CombinedEvent) -> Result<#struct_raw_name>;
        };
        let imp = quote! {
            #[allow(non_camel_case_types)]
            pub struct #topic_name(rustdds::Topic);

            impl Ros2Node {
                #[allow(non_snake_case)]
                pub fn #create_topic(&self, name_space: &str, base_name: &str, qos: ffi::Ros2QosPolicies) -> eyre::Result<Box<#topic_name>> {
                    let name = crate::ros2_client::Name::new(name_space, base_name).map_err(|e| eyre::eyre!(e))?;
                    let type_name = crate::ros2_client::MessageTypeName::new(#package_name, #self_name);
                    let topic = self.node.create_topic(&name, type_name, &qos.into())?;
                    Ok(Box::new(#topic_name(topic)))
                }

                #[allow(non_snake_case)]
                pub fn #create_publisher(&mut self, topic: &Box<#topic_name>, qos: ffi::Ros2QosPolicies) -> eyre::Result<Box<#publisher_name>> {
                    let publisher = self.node.create_publisher(&topic.0, Some(qos.into()))?;
                    Ok(Box::new(#publisher_name(publisher)))
                }

                #[allow(non_snake_case)]
                pub fn #create_subscription(&mut self, topic: &Box<#topic_name>, qos: ffi::Ros2QosPolicies, events: &mut crate::ffi::CombinedEvents) -> eyre::Result<Box<#subscription_name>> {
                    let subscription = self.node.create_subscription::<ffi::#struct_raw_name>(&topic.0, Some(qos.into()))?;
                    let stream = futures_lite::stream::unfold(subscription, |sub| async {
                        let item = sub.async_take().await;
                        let item_boxed: Box<dyn std::any::Any + 'static> = Box::new(item);
                        Some((item_boxed, sub))
                    });
                    let id = events.events.merge(Box::pin(stream));

                    Ok(Box::new(#subscription_name { id }))
                }
            }

            #[allow(non_camel_case_types)]
            pub struct #publisher_name(crate::ros2_client::Publisher<ffi::#struct_raw_name>);

            impl #publisher_name {
                #[allow(non_snake_case)]
                fn #publish(&mut self, message: ffi::#struct_raw_name) -> eyre::Result<()> {
                    use eyre::Context;
                    self.0.publish(message).context("publish failed").map_err(|e| eyre::eyre!("{e:?}"))
                }
            }

            #[allow(non_camel_case_types)]
            pub struct #subscription_name {
                id: u32,
            }

            impl #subscription_name {
                #[allow(non_snake_case)]
                fn #matches(&self, event: &crate::ffi::CombinedEvent) -> bool {
                    match &event.event.as_ref().0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.id  => true,
                        _ => false
                    }
                }
                #[allow(non_snake_case)]
                fn #downcast(&self, event: crate::ffi::CombinedEvent) -> eyre::Result<ffi::#struct_raw_name> {
                    use eyre::WrapErr;

                    match (*event.event).0 {
                        Some(crate::MergedEvent::External(event)) if event.id == self.id  => {
                            let result = event.event.downcast::<rustdds::dds::result::ReadResult<(ffi::#struct_raw_name, crate::ros2_client::MessageInfo)>>()
                                .map_err(|_| eyre::eyre!("downcast to {} failed", #struct_raw_name_str))?;

                            let (data, _info) = result.with_context(|| format!("failed to receive {} event", #subscription_name_str)).map_err(|e| eyre::eyre!("{e:?}"))?;
                            Ok(data)
                        },
                        _ => eyre::bail!("not a {} event", #subscription_name_str),
                    }
                }
            }
        };
        (def, imp)
    }

    pub fn alias_token_stream(&self, package_name: &Ident) -> impl ToTokens {
        let cxx_name = format_ident!("{}", self.name);
        let struct_raw_name = format_ident!("{package_name}__{}", self.name);

        if self.members.is_empty() {
            quote! {}
        } else {
            quote! {
                pub use super::super::ffi::#struct_raw_name as #cxx_name;
            }
        }
    }

    pub fn token_stream(&self) -> impl ToTokens {
        self.token_stream_args(false)
    }

    pub fn token_stream_args(&self, gen_cxx_bridge: bool) -> impl ToTokens {
        let rust_type = format_ident!("{}", self.name);
        let raw_type = format_ident!("{}_Raw", self.name);
        let raw_ref_type = format_ident!("{}_RawRef", self.name);

        let members_for_c = if self.members.is_empty() {
            vec![Member::dummy()]
        } else {
            self.members.clone()
        };

        let attributes = if gen_cxx_bridge {
            let namespace = &self.name;
            quote! { #[cxx::bridge(namespace = #namespace)] }
        } else {
            quote! {}
        };

        let rust_type_def_inner = self.members.iter().map(|m| m.rust_type_def(&self.package));
        let constants_def_inner = self.constants.iter().map(|c| c.token_stream());
        let rust_type_default_inner = self.members.iter().map(|m| m.default_value());

        let raw_type_def_inner = members_for_c.iter().map(|m| m.raw_type_def(&self.package));
        let raw_type_to_rust_inner = self.members.iter().map(|m| m.ffi_to_rust());

        let raw_ref_type_def_inner = members_for_c
            .iter()
            .map(|m| m.raw_ref_type_def(&self.package));

        let raw_ref_type_from_rust_inner = if self.members.is_empty() {
            vec![quote! { structure_needs_at_least_one_member: 0, }]
        } else {
            self.members
                .iter()
                .map(|m| {
                    let token = m.ffi_from_rust();
                    quote! { #token }
                })
                .collect::<Vec<_>>()
        };

        quote! {
            #[allow(unused_imports)]
            use std::convert::TryInto as _;
            use std::os::raw::c_void;

            use crate::_core::{
                InternalDefault as _,
                FFIFromRust as _FFIFromRust,
                FFIToRust as _FFIToRust,
            };

            pub use self::t::#rust_type;

            #attributes
            mod t {
                #[allow(non_camel_case_types)]
                #[derive(std::fmt::Debug, std::clone::Clone, std::cmp::PartialEq, serde::Serialize, serde::Deserialize)]
                pub struct #rust_type {
                    #(#rust_type_def_inner)*
                }
            }

            impl #rust_type {
                #(#constants_def_inner)*
            }


            impl crate::_core::MessageT for #rust_type {
                type Raw = #raw_type;
                type RawRef = #raw_ref_type;


            }

            impl crate::_core::InternalDefault for #rust_type {
                fn _default() -> Self {
                    Self {
                        #(#rust_type_default_inner)*
                    }
                }
            }

            impl std::default::Default for #rust_type {
                #[inline]
                fn default() -> Self {
                    crate::_core::InternalDefault::_default()
                }
            }


            #[allow(non_camel_case_types)]
            #[repr(C)]
            #[derive(std::fmt::Debug)]
            pub struct #raw_type {
                #(#raw_type_def_inner)*
            }

            impl crate::_core::FFIToRust for #raw_type {
                type Target = #rust_type;

                unsafe fn to_rust(&self) -> Self::Target {
                    Self::Target {
                        #(#raw_type_to_rust_inner)*
                    }
                }
            }

            unsafe impl std::marker::Send for #raw_type {}
            unsafe impl std::marker::Sync for #raw_type {}

            #[allow(non_camel_case_types)]
            #[doc(hidden)]
            #[repr(C)]
            #[derive(std::fmt::Debug)]
            pub struct #raw_ref_type {
                #(#raw_ref_type_def_inner)*
            }

            impl crate::_core::FFIFromRust for #raw_ref_type {
                type From = #rust_type;

                #[allow(unused_variables)]
                unsafe fn from_rust(from: &Self::From) -> Self {
                    Self {
                        #(#raw_ref_type_from_rust_inner)*
                    }
                }
            }

            #[cfg(test)]
            mod test {
                use super::*;
                use crate::_core::MessageT;

                #[test]
                fn test_rust_default() {
                    let _ = #rust_type::default();
                }

                #[test]
                fn test_raw_default() {
                    let _ = #raw_type::default();
                }

                #[test]
                fn test_type_support() {
                    let ptr = #rust_type::type_support();
                    assert!(!ptr.is_null());
                }
            }

        }
    }
}

/// Keywords in Rust
///
/// <https://doc.rust-lang.org/reference/keywords.html>
const RUST_KEYWORDS: [&str; 51] = [
    // Strict keywords
    "as", "break", "const", "continue", "crate", "else", "enum", "extern", "false", "fn", "for",
    "if", "impl", "in", "let", "loop", "match", "mod", "move", "mut", "pub", "ref", "return",
    "self", "Self", "static", "struct", "super", "trait", "true", "type", "unsafe", "use", "where",
    "while", //
    // Strict keywords (2018+)
    "async", "await", "dyn", //
    // Reserved keywords
    "abstract", "become", "box", "do", "final", "macro", "override", "priv", "typeof", "unsized",
    "virtual", "yield", //
    // Reserved keywords (2018+)
    "try",
];
