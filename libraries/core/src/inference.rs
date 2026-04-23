//! Type inference from Rust source code (Phase 8).
//!
//! Scans Rust source files for `send_output` calls and maps Arrow constructor
//! patterns to type URNs. Best-effort: unrecognizable patterns produce no
//! suggestion (not an error).
//!
//! Feature-gated behind `type-inference`.

use std::path::Path;
use syn::{Expr, ExprMethodCall, visit::Visit};

/// A suggested type annotation inferred from source code.
#[derive(Debug)]
pub struct TypeSuggestion {
    /// Node ID (derived from context, not from source)
    pub node_id: String,
    /// Output ID found in `send_output` calls
    pub output_id: String,
    /// Suggested type URN
    pub suggested_urn: String,
    /// Source file path
    pub file: String,
    /// Line number in source (None if unavailable)
    pub line: Option<usize>,
}

impl std::fmt::Display for TypeSuggestion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(line) = self.line {
            write!(
                f,
                "suggest: node \"{}\" output \"{}\" -> {} (from {}:{})",
                self.node_id, self.output_id, self.suggested_urn, self.file, line,
            )
        } else {
            write!(
                f,
                "suggest: node \"{}\" output \"{}\" -> {} (from {})",
                self.node_id, self.output_id, self.suggested_urn, self.file,
            )
        }
    }
}

/// Scan a Rust source file for type hints. Returns suggestions.
pub fn infer_types_from_file(
    source_path: &Path,
    node_id: &str,
) -> Result<Vec<TypeSuggestion>, String> {
    let content = std::fs::read_to_string(source_path)
        .map_err(|e| format!("{}: {e}", source_path.display()))?;
    infer_types_from_source(&content, source_path.to_string_lossy().as_ref(), node_id)
}

/// Scan Rust source text for type hints.
pub fn infer_types_from_source(
    source: &str,
    file_name: &str,
    node_id: &str,
) -> Result<Vec<TypeSuggestion>, String> {
    let syntax =
        syn::parse_file(source).map_err(|e| format!("failed to parse {file_name}: {e}"))?;

    let mut visitor = TypeInferenceVisitor {
        suggestions: Vec::new(),
        node_id: node_id.to_string(),
        file_name: file_name.to_string(),
    };
    visitor.visit_file(&syntax);
    Ok(visitor.suggestions)
}

struct TypeInferenceVisitor {
    suggestions: Vec<TypeSuggestion>,
    node_id: String,
    file_name: String,
}

impl<'ast> Visit<'ast> for TypeInferenceVisitor {
    fn visit_expr_method_call(&mut self, node: &'ast ExprMethodCall) {
        let method_name = node.method.to_string();
        if method_name == "send_output" && node.args.len() >= 2 {
            // Try to extract output_id from first argument (string literal)
            if let Some(output_id) = extract_string_lit(&node.args[0]) {
                // Try to infer type from second argument
                if let Some(urn) = infer_type_from_expr(&node.args[1]) {
                    self.suggestions.push(TypeSuggestion {
                        node_id: self.node_id.clone(),
                        output_id,
                        suggested_urn: urn,
                        file: self.file_name.clone(),
                        line: None,
                    });
                }
            }
        }
        // Continue visiting nested expressions
        syn::visit::visit_expr_method_call(self, node);
    }
}

/// Try to extract a string literal from an expression.
fn extract_string_lit(expr: &Expr) -> Option<String> {
    match expr {
        Expr::Lit(lit) => match &lit.lit {
            syn::Lit::Str(s) => Some(s.value()),
            _ => None,
        },
        Expr::Reference(r) => extract_string_lit(&r.expr),
        _ => None,
    }
}

/// Try to infer a type URN from an expression (e.g. `Float64Array::from(...)`).
fn infer_type_from_expr(expr: &Expr) -> Option<String> {
    match expr {
        Expr::Call(call) => {
            // Look for Type::from(...) patterns
            if let Expr::Path(path) = &*call.func {
                let segments: Vec<String> = path
                    .path
                    .segments
                    .iter()
                    .map(|s| s.ident.to_string())
                    .collect();
                let full = segments.join("::");
                return map_constructor_to_urn(&full);
            }
            None
        }
        Expr::MethodCall(mc) => {
            // Look for .into() on typed values, or ArrowArray constructors
            let method = mc.method.to_string();
            if method == "into" || method == "from" {
                // Try to get type from receiver
                return infer_type_from_expr(&mc.receiver);
            }
            None
        }
        Expr::Path(path) => {
            let segments: Vec<String> = path
                .path
                .segments
                .iter()
                .map(|s| s.ident.to_string())
                .collect();
            let full = segments.join("::");
            map_constructor_to_urn(&full)
        }
        _ => None,
    }
}

/// Map known Arrow constructor names to type URNs.
fn map_constructor_to_urn(name: &str) -> Option<String> {
    match name {
        "Float32Array" | "Float32Array::from" => Some("std/core/v1/Float32".into()),
        "Float64Array" | "Float64Array::from" => Some("std/core/v1/Float64".into()),
        "Int32Array" | "Int32Array::from" => Some("std/core/v1/Int32".into()),
        "Int64Array" | "Int64Array::from" => Some("std/core/v1/Int64".into()),
        "UInt8Array" | "UInt8Array::from" => Some("std/core/v1/UInt8".into()),
        "UInt32Array" | "UInt32Array::from" => Some("std/core/v1/UInt32".into()),
        "UInt64Array" | "UInt64Array::from" => Some("std/core/v1/UInt64".into()),
        "StringArray" | "StringArray::from" => Some("std/core/v1/String".into()),
        "BooleanArray" | "BooleanArray::from" => Some("std/core/v1/Bool".into()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn infer_float64_from_send_output() {
        let source = r#"
            fn main() {
                node.send_output("reading", Float64Array::from(vec![42.0]));
            }
        "#;
        let suggestions = infer_types_from_source(source, "test.rs", "sensor").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].output_id, "reading");
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/Float64");
        assert_eq!(suggestions[0].node_id, "sensor");
    }

    #[test]
    fn infer_unrecognizable_no_suggestion() {
        let source = r#"
            fn main() {
                node.send_output("data", some_complex_expression());
            }
        "#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert!(suggestions.is_empty());
    }

    #[test]
    fn infer_no_send_output() {
        let source = r#"
            fn main() {
                let x = 42;
            }
        "#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert!(suggestions.is_empty());
    }

    #[test]
    fn infer_string_array() {
        let source = r#"
            fn main() {
                node.send_output("text", StringArray::from(vec!["hello"]));
            }
        "#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/String");
    }
}
