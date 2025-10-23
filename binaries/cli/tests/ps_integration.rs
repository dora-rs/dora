use dora_cli::cli::{commands::ps::PsCommand, context::ExecutionContext};

#[tokio::test]
async fn test_ps_command_basic() {
    let context = ExecutionContext::mock_interactive();
    let cmd = PsCommand::default();
    
    // Should execute without error
    let result = cmd.execute(&context).await;
    assert!(result.is_ok());
}

#[tokio::test] 
async fn test_ps_command_json_output() {
    let context = ExecutionContext::mock_non_interactive();
    let cmd = PsCommand {
        format: Some(dora_cli::cli::OutputFormat::Json),
        ..Default::default()
    };
    
    let result = cmd.execute(&context).await;
    assert!(result.is_ok());
}

#[tokio::test]
async fn test_ps_filtering() {
    let context = ExecutionContext::mock_interactive();
    let cmd = PsCommand {
        status: Some(dora_cli::cli::commands::ps::DataflowStatusFilter::Running),
        ..Default::default()
    };
    
    let result = cmd.execute(&context).await;
    assert!(result.is_ok());
}

#[test]
fn test_dataflow_filtering_unit() {
    let cmd = PsCommand {
        status: Some(dora_cli::cli::commands::ps::DataflowStatusFilter::Running),
        ..Default::default()
    };
    
    let dataflows = cmd.create_mock_dataflows();
    let filtered = cmd.apply_filters(&dataflows);
    
    // Should only show running dataflows
    assert!(filtered.iter().all(|df| df.status == dora_cli::cli::commands::ps::DataflowStatus::Running));
}

#[test]
fn test_dataflow_sorting_unit() {
    let cmd = PsCommand {
        sort: dora_cli::cli::commands::ps::SortField::Name,
        reverse: false,
        ..Default::default()
    };
    
    let dataflows = cmd.create_mock_dataflows();
    let sorted = cmd.apply_sorting(dataflows);
    
    // Should be sorted by name alphabetically
    assert_eq!(sorted[0].name, "chat-demo");
    assert_eq!(sorted[1].name, "object-detection");
    assert_eq!(sorted[2].name, "simple-test");
}