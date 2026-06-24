//! Type inference from Rust source code (Phase 8).
//!
//! Scans Rust source files for `send_output` calls and maps Arrow constructor
//! patterns to type URNs. Best-effort: unrecognizable patterns produce no
//! suggestion (not an error).
//!
//! Feature-gated behind `type-inference`.

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
        // The real Rust node API signature is:
        //   send_output(output_id, parameters: MetadataParameters, data: impl Array)
        // so the data (Arrow array) is at argument index 2, not 1.
        if method_name == "send_output" && node.args.len() >= 3 {
            // Try to extract output_id from first argument (string literal)
            if let Some(output_id) = extract_string_lit(&node.args[0]) {
                // Try to infer type from third argument (the Arrow data array)
                if let Some(urn) = infer_type_from_expr(&node.args[2]) {
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
        // Recognize the idiomatic `"output_id".into()` form used by every
        // real send_output call site, where the first argument is a DataId
        // constructed via `Into<DataId>` from a string literal.
        Expr::MethodCall(mc) if mc.method == "into" => extract_string_lit(&mc.receiver),
        _ => None,
    }
}

/// Try to infer a type URN from an expression (e.g. `Float64Array::from(...)`
/// or `value.into_arrow()`).
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
            let method = mc.method.to_string();
            if method == "into" || method == "from" {
                // Try to get type from receiver
                return infer_type_from_expr(&mc.receiver);
            }
            // Recognise the idiomatic `.into_arrow()` call: the type information
            // lives in the receiver (e.g. `Float64Array::from(...).into_arrow()`
            // or a named variable — the latter we cannot resolve statically, but
            // we can still handle the constructor chain form).
            if method == "into_arrow" {
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
        "Int8Array" | "Int8Array::from" => Some("std/core/v1/Int8".into()),
        "Int16Array" | "Int16Array::from" => Some("std/core/v1/Int16".into()),
        "Int32Array" | "Int32Array::from" => Some("std/core/v1/Int32".into()),
        "Int64Array" | "Int64Array::from" => Some("std/core/v1/Int64".into()),
        "UInt8Array" | "UInt8Array::from" => Some("std/core/v1/UInt8".into()),
        "UInt16Array" | "UInt16Array::from" => Some("std/core/v1/UInt16".into()),
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

    // ── primary fix: 3-argument real API ────────────────────────────────────

    #[test]
    fn infer_float64_from_send_output_real_api() {
        // Uses the actual 3-arg dora Rust API signature:
        //   send_output(output_id, MetadataParameters::default(), data)
        let source = r#"
fn main() {
    node.send_output(
        "reading",
        MetadataParameters::default(),
        Float64Array::from(vec![42.0]),
    );
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "sensor").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].output_id, "reading");
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/Float64");
        assert_eq!(suggestions[0].node_id, "sensor");
    }

    #[test]
    fn infer_from_metadata_variable() {
        // As seen in examples/multiple-daemons/node/src/main.rs
        let source = r#"
fn main() {
    node.send_output(output.clone(), metadata.parameters, random.into_arrow())?;
}
"#;
        // `random.into_arrow()` has an opaque receiver — no static type info available.
        // Expectation: no suggestion (graceful no-op, not a panic).
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert!(suggestions.is_empty());
    }

    #[test]
    fn infer_from_real_call_site_idiom() {
        // Mirrors the real idiom used throughout examples/, e.g.
        // examples/error-propagation/producer/src/main.rs:
        //   node.send_output("data".into(), MetadataParameters::default(), i.into_arrow())?;
        // The output_id is `"data".into()` (a DataId via Into<DataId>), not a
        // bare string literal.
        let source = r#"
fn main() {
    node.send_output(
        "data".into(),
        MetadataParameters::default(),
        Int64Array::from(vec![i]).into_arrow(),
    )?;
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "producer").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].output_id, "data");
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/Int64");
    }

    // ── into_arrow() chain: type visible in the constructor ─────────────────

    #[test]
    fn infer_via_into_arrow_chain() {
        let source = r#"
fn main() {
    node.send_output(
        "sensor",
        MetadataParameters::default(),
        Float32Array::from(vec![1.0_f32]).into_arrow(),
    );
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/Float32");
    }

    // ── new type mappings ───────────────────────────────────────────────────

    #[test]
    fn infer_int8_array() {
        let source = r#"
fn main() {
    node.send_output("out", MetadataParameters::default(), Int8Array::from(vec![1_i8]));
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/Int8");
    }

    #[test]
    fn infer_int16_array() {
        let source = r#"
fn main() {
    node.send_output("out", MetadataParameters::default(), Int16Array::from(vec![1_i16]));
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/Int16");
    }

    #[test]
    fn infer_uint16_array() {
        let source = r#"
fn main() {
    node.send_output("out", MetadataParameters::default(), UInt16Array::from(vec![1_u16]));
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/UInt16");
    }

    // ── unchanged behaviour ─────────────────────────────────────────────────

    #[test]
    fn infer_unrecognizable_no_suggestion() {
        let source = r#"
fn main() {
    node.send_output("data", MetadataParameters::default(), some_complex_expression());
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
    node.send_output("text", MetadataParameters::default(), StringArray::from(vec!["hello"]));
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "node").unwrap();
        assert_eq!(suggestions.len(), 1);
        assert_eq!(suggestions[0].suggested_urn, "std/core/v1/String");
    }

    // ── two-arg call (old fictional form) should NOT fire ───────────────────

    #[test]
    fn two_arg_send_output_does_not_fire() {
        // The old tests used this fictional 2-arg form; ensure it no longer
        // produces a false positive now that we require >= 3 args.
        let source = r#"
fn main() {
    node.send_output("reading", Float64Array::from(vec![42.0]));
}
"#;
        let suggestions = infer_types_from_source(source, "test.rs", "sensor").unwrap();
        assert!(
            suggestions.is_empty(),
            "2-arg form should not match the real 3-arg API"
        );
    }
}
