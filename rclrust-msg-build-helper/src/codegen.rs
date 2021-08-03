use rclrust_msg_types as types;

/// Keywords in Rust
/// https://doc.rust-lang.org/reference/keywords.html
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

pub fn escape_keyword(s: &str) -> String {
    if RUST_KEYWORDS.contains(&s) {
        format!("r#{}", s)
    } else {
        s.into()
    }
}

pub const RCLRS_MSG_CORE: &str = "rclrust_msg_core";
pub const MSG_TYPE_SUPPORT_PREFIX: &str = "rosidl_typesupport_c__get_message_type_support_handle";
pub const SRV_TYPE_SUPPORT_PREFIX: &str = "rosidl_typesupport_c__get_service_type_support_handle";
pub const ACTION_TYPE_SUPPORT_PREFIX: &str = "rosidl_typesupport_c__get_action_type_support_handle";

pub fn create_zero_init_str(member: &types::Member) -> String {
    match member.r#type {
        types::MemberType::Array(ref t) => match t.value_type {
            types::NestableType::BasicType(_) => {
                format!("[_ZeroInit::zero_init(); {}]", t.size)
            }
            _ => {
                format!("[{}]", vec!["_ZeroInit::zero_init()"; t.size].join(", "))
            }
        },
        _ => "_ZeroInit::zero_init()".into(),
    }
}

const ITER_TO_ARRAY: &str = ".collect::<Vec<_>>().try_into().unwrap()";

pub fn create_ffi_to_rust(member: &types::Member) -> String {
    let name = escape_keyword(&member.name);
    match member.r#type {
        types::MemberType::BasicType(_) => format!("self.{}", name),
        types::MemberType::Array(ref t) => match t.value_type {
            types::NestableType::BasicType(_) => format!("self.{}.clone()", name),
            _ => format!(
                "self.{}.iter().map(_FFIToRust::to_rust){}",
                name, ITER_TO_ARRAY
            ),
        },
        _ => format!("self.{}.to_rust()", name),
    }
}

pub fn create_ffi_from_rust(member: &types::Member) -> String {
    let name = escape_keyword(&member.name);
    match member.r#type {
        types::MemberType::BasicType(_) => format!("from.{}", name),
        types::MemberType::Array(ref t) => match t.value_type {
            types::NestableType::BasicType(_) => format!("from.{}.clone()", name),
            _ => format!(
                "from.{}.iter().map(_FFIFromRust::from_rust){}",
                name, ITER_TO_ARRAY
            ),
        },
        _ => format!("_FFIFromRust::from_rust(&from.{})", name),
    }
}

pub fn msg_type_to_ffi_raw(member: &types::Member, pkg_name: &str) -> String {
    let rs_inner_type = match member.r#type.clone().inner_type() {
        types::NestableType::BasicType(ref t) => t.to_rust_str().into(),
        types::NestableType::NamedType(ref t) => format!("crate::{}::msg::{}_Raw", pkg_name, t.0),
        types::NestableType::NamespacedType(ref t) => {
            format!("crate::{}_Raw", t.to_rust_str())
        }
        types::NestableType::GenericString(ref t) => {
            if t.is_wide() {
                format!("{}::FFIWString", RCLRS_MSG_CORE)
            } else {
                format!("{}::FFIString", RCLRS_MSG_CORE)
            }
        }
    };

    match member.r#type {
        types::MemberType::Array(ref t) => format!("[{}; {}]", rs_inner_type, t.size),
        types::MemberType::Sequence(_) | types::MemberType::BoundedSequence(_) => {
            format!("{}::FFISeq<{}>", RCLRS_MSG_CORE, rs_inner_type)
        }
        _ => rs_inner_type,
    }
}

pub fn msg_type_to_ffi_raw_ref(member: &types::Member, pkg_name: &str) -> String {
    let rs_inner_type = match member.r#type.clone().inner_type() {
        types::NestableType::BasicType(ref t) => t.to_rust_str().into(),
        types::NestableType::NamedType(ref t) => {
            format!("crate::{}::msg::{}_RawRef", pkg_name, t.0)
        }
        types::NestableType::NamespacedType(ref t) => {
            format!("crate::{}_RawRef", t.to_rust_str())
        }
        types::NestableType::GenericString(ref t) => {
            if t.is_wide() {
                format!("{}::OwnedFFIWString", RCLRS_MSG_CORE)
            } else {
                format!("{}::OwnedFFIString", RCLRS_MSG_CORE)
            }
        }
    };

    match member.r#type {
        types::MemberType::Array(ref t) => format!("[{}; {}]", rs_inner_type, t.size),
        types::MemberType::Sequence(types::Sequence { ref value_type })
        | types::MemberType::BoundedSequence(types::BoundedSequence { ref value_type, .. }) => {
            match value_type {
                types::NestableType::BasicType(_) => {
                    format!("{}::RefFFISeq<{}>", RCLRS_MSG_CORE, rs_inner_type)
                }
                _ => format!("{}::OwnedFFISeq<{}>", RCLRS_MSG_CORE, rs_inner_type),
            }
        }
        _ => rs_inner_type,
    }
}

pub fn msg_type_to_rs_not_raw(member: &types::Member, pkg_name: &str) -> String {
    let rs_inner_type = match member.r#type.clone().inner_type() {
        types::NestableType::BasicType(ref t) => t.to_rust_str().into(),
        types::NestableType::NamedType(ref t) => format!("crate::{}::msg::{}", pkg_name, t.0),
        types::NestableType::NamespacedType(ref t) => {
            format!("crate::{}", t.to_rust_str())
        }
        types::NestableType::GenericString(ref t) => {
            if t.is_wide() {
                "crate::widestring::U16String".into()
            } else {
                "std::string::String".into()
            }
        }
    };

    match member.r#type {
        types::MemberType::Array(ref t) => format!("[{}; {}]", rs_inner_type, t.size),
        types::MemberType::Sequence(_) | types::MemberType::BoundedSequence(_) => {
            format!("std::vec::Vec<{}>", rs_inner_type)
        }
        _ => rs_inner_type,
    }
}

pub fn constant_type_str(constant: &types::Constant) -> String {
    match constant.r#type {
        types::ConstantType::BasicType(ref t) => t.to_rust_str().into(),
        types::ConstantType::GenericUnboundedString(_) => "&str".into(),
        types::ConstantType::PrimitiveArray(ref array_t) => match array_t.value_type {
            types::PrimitiveType::BasicType(ref t) => {
                format!("[{}; {}]", t.to_rust_str(), array_t.size)
            }
            types::PrimitiveType::GenericUnboundedString(_) => {
                format!("[&str; {}]", array_t.size)
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_escape_keyword() {
        assert_eq!(escape_keyword("type"), "r#type");
        assert_eq!(escape_keyword("type2"), "type2");
    }
}
