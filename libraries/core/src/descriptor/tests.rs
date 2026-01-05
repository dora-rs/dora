#[cfg(test)]
mod tests {
    use crate::descriptor::{resolve_descriptor_from_path, load_descriptor_with_metadata};
    use std::path::Path;
    use dora_message::id::NodeId;

    #[test]
    fn test_legacy_format() {
        let dataflow_path = Path::new("examples/python-log/dataflow.yaml");
        if dataflow_path.exists() {
            let result = resolve_descriptor_from_path(dataflow_path);
            assert!(result.is_ok(), "Failed to parse legacy format: {:?}", result.err());
            
            let resolved = result.unwrap();
            assert_eq!(resolved.len(), 2, "Expected 2 nodes in legacy format");
            
            assert!(resolved.contains_key(&NodeId::from("send_data".to_string())));
            assert!(resolved.contains_key(&NodeId::from("receive_data_with_sleep".to_string())));
        }
    }

    #[test]
    fn test_new_format() {
        let dataflow_path = Path::new("examples/python-log/dataflow 2.yaml");
        let metadata_path = Path::new("examples/python-log/dora.yaml");
        
        if dataflow_path.exists() && metadata_path.exists() {
            let result = resolve_descriptor_from_path(dataflow_path);
            assert!(result.is_ok(), "Failed to parse new format: {:?}", result.err());
            
            let resolved = result.unwrap();
            assert_eq!(resolved.len(), 2, "Expected 2 nodes in new format");
            
            assert!(resolved.contains_key(&NodeId::from("send_data".to_string())));
            assert!(resolved.contains_key(&NodeId::from("receive_data_with_sleep".to_string())));
            
            // Verify that the nodes have the correct names from prototypes
            let send_node = &resolved[&NodeId::from("send_data".to_string())];
            assert_eq!(send_node.name, Some("sender".to_string()));
            
            let receive_node = &resolved[&NodeId::from("receive_data_with_sleep".to_string())];
            assert_eq!(receive_node.name, Some("receiver".to_string()));
        }
    }

    #[test]
    fn test_load_with_metadata() {
        let dataflow_path = Path::new("examples/python-log/dataflow 2.yaml");
        
        if dataflow_path.exists() {
            let result = load_descriptor_with_metadata(dataflow_path);
            assert!(result.is_ok(), "Failed to load with metadata: {:?}", result.err());
            
            let (descriptor, metadata) = result.unwrap();
            assert!(!descriptor.graph.is_empty(), "Graph should not be empty");
            assert!(metadata.is_some(), "Metadata should be loaded");
            
            if let Some(meta) = metadata {
                assert_eq!(meta.nodes.len(), 3, "Expected 3 node prototypes");
            }
        }
    }
}
