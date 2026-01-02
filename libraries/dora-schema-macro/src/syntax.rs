use std::collections::HashSet;

use syn::{
    parse::{Parse, ParseStream},
    Ident, Result, Token,
};

#[derive(Debug)]
pub struct SchemaInput {
    pub client_name: Ident,
    pub server_name: Ident,
    pub methods: Vec<MethodDef>,
}

impl SchemaInput {
    pub fn protocol_name(&self) -> String {
        format!("{}To{}", self.client_name, self.server_name)
    }
}

#[derive(Debug)]
pub struct MethodDef {
    /// Method name, should be in snake_case
    pub name: Ident,
    /// Request type
    pub request: Ident,
    /// Response type
    pub response: Ident,
}

impl Parse for SchemaInput {
    fn parse(input: ParseStream) -> Result<Self> {
        // Cli => Coordinator:
        let client_name: Ident = input.parse()?;
        input.parse::<Token![=>]>()?;
        let server_name: Ident = input.parse()?;
        input.parse::<Token![:]>()?;

        let mut methods = Vec::new();
        let mut methods_seen = HashSet::new();
        while !input.is_empty() {
            let method = input.parse::<MethodDef>()?;
            if !methods_seen.insert(method.name.to_string()) {
                return Err(input.error(format!("Duplicate method name: {}", method.name)));
            }
            methods.push(method);
        }

        Ok(SchemaInput {
            client_name,
            server_name,
            methods,
        })
    }
}

impl Parse for MethodDef {
    fn parse(input: ParseStream) -> Result<Self> {
        // list_nodes: ListNodesRequest => ListNodesResponse;
        let name: Ident = input
            .parse()
            .map_err(|_| input.error("Expected method name (e.g., list_nodes)".to_string()))?;
        input.parse::<Token![:]>()?;
        let request: Ident = input.parse().map_err(|_| {
            input.error("Expected request type (e.g., ListNodesRequest)".to_string())
        })?;
        input.parse::<Token![=>]>()?;
        let response: Ident = input.parse().map_err(|_| {
            input.error("Expected response type (e.g., ListNodesResponse)".to_string())
        })?;
        input.parse::<Token![;]>()?;

        Ok(MethodDef {
            name,
            request,
            response,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use quote::quote;
    use syn::parse2;

    #[test]
    fn parse_schema_input_valid_two_methods() {
        let input = quote! {
            SomeClient => SomeServer:
            list_nodes: ListNodesRequest => ListNodesResponse;
            get_node: GetNodeRequest => GetNodeResponse;
        };

        let schema: SchemaInput = parse2(input).expect("Failed to parse schema input");

        assert_eq!(schema.client_name.to_string(), "SomeClient");
        assert_eq!(schema.server_name.to_string(), "SomeServer");
        assert_eq!(schema.methods.len(), 2);

        assert_eq!(schema.methods[0].name.to_string(), "list_nodes");
        assert_eq!(schema.methods[0].request.to_string(), "ListNodesRequest");
        assert_eq!(schema.methods[0].response.to_string(), "ListNodesResponse");

        assert_eq!(schema.methods[1].name.to_string(), "get_node");
        assert_eq!(schema.methods[1].request.to_string(), "GetNodeRequest");
        assert_eq!(schema.methods[1].response.to_string(), "GetNodeResponse");
    }

    #[test]
    fn parse_schema_input_no_methods() {
        let input = quote! {
            SomeClient => SomeServer:
        };

        let schema: SchemaInput =
            parse2(input).expect("Failed to parse schema input with no methods");
        assert_eq!(schema.client_name.to_string(), "SomeClient");
        assert_eq!(schema.server_name.to_string(), "SomeServer");
        assert!(schema.methods.is_empty());
    }

    #[test]
    fn parse_schema_input_syntax_error() {
        let bad_input = quote! {
            SomeClient SomeServer:
        };
        let _ = parse2::<SchemaInput>(bad_input).expect_err("Expected parsing to fail");
    }

    #[test]
    fn parse_schema_input_duplicate_method_error() {
        let dup_method_input = quote! {
            SomeClient => SomeServer:
            list_nodes: ListNodesRequest => ListNodesResponse;
            list_nodes: GetNodeRequest => GetNodeResponse;
        };
        let _ = parse2::<SchemaInput>(dup_method_input).expect_err("Expected parsing to fail");
    }

    #[test]
    fn test_parse_method_def() {
        let input = quote! {
            list_nodes: ListNodesRequest => ListNodesResponse;
        };

        let method: MethodDef = parse2(input).expect("Failed to parse method definition");

        assert_eq!(method.name.to_string(), "list_nodes");
        assert_eq!(method.request.to_string(), "ListNodesRequest");
        assert_eq!(method.response.to_string(), "ListNodesResponse");
    }
}
