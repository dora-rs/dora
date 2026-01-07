use dora_core::descriptor::{DescriptorExt, NodeMetadataFileExt};
use std::path::Path;

fn main() -> eyre::Result<()> {
    // Test parsing old format
    println!("=== Testing old format (dataflow.yaml) ===");
    let old_dataflow = dora_core::descriptor::Descriptor::blocking_read(Path::new("dataflow.yaml"))?;
    println!("Old format parsed successfully!");
    println!("Number of nodes: {}", old_dataflow.nodes.len());
    println!("Number of graph nodes: {}", old_dataflow.graph.len());
    
    let resolved_old = old_dataflow.resolve_aliases_and_set_defaults()?;
    println!("Resolved {} nodes", resolved_old.len());
    for (id, node) in &resolved_old {
        println!("  - {}: {:?}", id, node.name);
    }

    // Test parsing new format
    println!("\n=== Testing new format (dataflow 2.yaml + dora.yaml) ===");
    let new_dataflow = dora_core::descriptor::Descriptor::blocking_read(Path::new("dataflow 2.yaml"))?;
    println!("New format parsed successfully!");
    println!("Number of nodes: {}", new_dataflow.nodes.len());
    println!("Number of graph nodes: {}", new_dataflow.graph.len());
    
    // Load metadata
    let metadata = dora_core::descriptor::NodeMetadataFile::blocking_read(Path::new("dora.yaml"))?;
    println!("Metadata file parsed successfully!");
    println!("Number of node prototypes: {}", metadata.nodes.len());
    for proto in &metadata.nodes {
        println!("  - {} (lang: {:?})", proto.name, proto.lang);
    }
    
    // Resolve with metadata
    let resolved_new = new_dataflow.resolve_with_metadata(Some(&metadata))?;
    println!("\nResolved {} nodes", resolved_new.len());
    for (id, node) in &resolved_new {
        println!("  - {}: {:?}", id, node.name);
    }

    println!("\n=== All tests passed! ===");
    Ok(())
}
