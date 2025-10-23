# Issue #001: Implement Hybrid Command Framework with Clap

## 📋 Summary
Implement the foundational CLI command structure using clap that supports both traditional CLI commands and TUI mode selection. This framework will serve as the base for all subsequent CLI enhancements.

## 🎯 Objectives
- Create a three-tier command structure (Docker-like, Enhanced, TUI modes)
- Implement global flags for UI mode control
- Add output format selection
- Establish the foundation for smart interface selection
- Ensure backward compatibility with existing commands

**Success Metrics:**
- All existing CLI commands continue to work without modification
- New global flags (`--ui-mode`, `--output`, `--no-hints`) function correctly
- Command parsing handles all three tiers appropriately
- Foundation supports future interface selection logic

## 🛠️ Technical Requirements

### What to Build

#### 1. Enhanced CLI Structure
```rust
// src/cli/mod.rs
#[derive(Parser)]
#[clap(name = "dora", about = "Dora dataflow runtime")]
pub struct Cli {
    #[clap(subcommand)]
    pub command: Command,
    
    /// Force interface mode
    #[clap(long, global = true, value_enum)]
    pub ui_mode: Option<UiMode>,
    
    /// Output format
    #[clap(long, global = true, value_enum, default_value = "auto")]
    pub output: OutputFormat,
    
    /// Disable TUI hints and prompts
    #[clap(long, global = true)]
    pub no_hints: bool,
}
```

#### 2. Three-Tier Command Enum
```rust
#[derive(Subcommand)]
pub enum Command {
    // Tier 1: Core Docker-like commands
    #[clap(alias = "list")]
    Ps(PsCommand),
    Start(StartCommand), 
    Stop(StopCommand),
    Logs(LogsCommand),
    Build(BuildCommand),
    
    // Tier 2: Enhanced commands with smart suggestions
    Inspect(InspectCommand),
    Debug(DebugCommand),
    Analyze(AnalyzeCommand),
    Monitor(MonitorCommand),
    
    // Tier 3: Explicit TUI modes
    Ui(UiCommand),
    
    // System management
    System(SystemCommand),
    Config(ConfigCommand),
}
```

#### 3. Mode and Format Enums
```rust
#[derive(Clone, Debug, clap::ValueEnum)]
pub enum UiMode {
    Auto,     // Smart decisions based on context
    Cli,      // Force CLI output
    Tui,      // Force TUI when available
    Minimal,  // Minimal output (CI/scripting)
}

#[derive(Clone, Debug, clap::ValueEnum)]  
pub enum OutputFormat {
    Auto,     // Context-appropriate formatting
    Table,    // Human-readable table
    Json,     // Machine-readable JSON
    Yaml,     // YAML format
    Minimal,  // Minimal text output
}
```

### Why This Approach

**Clap Parser Benefits:**
- Automatic help generation with examples
- Type-safe argument parsing
- Built-in validation and error messages
- Easy extensibility for future commands
- Excellent ecosystem support

**Three-Tier Architecture Benefits:**
- Clear separation of complexity levels
- Progressive disclosure of advanced features
- Maintains Docker-like simplicity for basic operations
- Provides explicit control for power users

**Global Flags Strategy:**
- Consistent behavior across all commands
- User preference overrides at any level
- CI/automation friendly with `--no-hints` and `minimal` mode
- Future-proof for additional global options

### How to Implement

#### Step 1: Update Dependencies (30 minutes)
```toml
# Add to Cargo.toml
[dependencies]
clap = { version = "4.4", features = ["derive", "env"] }
clap_complete = "4.4"  # For shell completion
serde = { version = "1.0", features = ["derive"] }
```

#### Step 2: Create Base CLI Structure (2 hours)
1. **Create `src/cli/mod.rs`** with the main `Cli` struct
2. **Define command enums** with proper clap attributes
3. **Add mode and format enums** with value_enum derive
4. **Implement basic parsing** with error handling

#### Step 3: Migrate Existing Commands (4 hours)
1. **Refactor existing command structs** to use new patterns
2. **Add backward compatibility aliases** (`list` -> `ps`)
3. **Implement command execution traits** for consistency
4. **Update main.rs** to use new CLI structure

#### Step 4: Add Global Flag Handling (2 hours)
1. **Create context detection** for global flags
2. **Implement output formatting** dispatch
3. **Add hint suppression** logic
4. **Create mode validation** with appropriate defaults

#### Step 5: Testing and Validation (2 hours)
1. **Test all existing commands** still work
2. **Validate new global flags** function correctly
3. **Check help text generation** is comprehensive
4. **Verify error messages** are helpful and actionable

## 🔗 Dependencies
**Blocks:** Issues #002, #003, #004 (need this foundation)
**No Dependencies:** This is the foundational issue

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_cli_parsing() {
        // Test basic command parsing
        let cli = Cli::try_parse_from(["dora", "ps"]).unwrap();
        assert!(matches!(cli.command, Command::Ps(_)));
    }
    
    #[test]
    fn test_global_flags() {
        // Test UI mode flag
        let cli = Cli::try_parse_from(["dora", "--ui-mode", "cli", "ps"]).unwrap();
        assert_eq!(cli.ui_mode, Some(UiMode::Cli));
    }
    
    #[test]
    fn test_output_format() {
        // Test output format flag
        let cli = Cli::try_parse_from(["dora", "--output", "json", "ps"]).unwrap();
        assert_eq!(cli.output, OutputFormat::Json);
    }
    
    #[test]
    fn test_backward_compatibility() {
        // Test aliases work
        let cli = Cli::try_parse_from(["dora", "list"]).unwrap();
        assert!(matches!(cli.command, Command::Ps(_)));
    }
}
```

### Integration Tests
```rust
// tests/cli_integration.rs
#[test]
fn test_help_generation() {
    // Verify help text is comprehensive
    let output = Command::new("dora").try_get_matches_from(["dora", "--help"]);
    // Validate help contains expected sections
}

#[test]
fn test_error_handling() {
    // Test invalid arguments produce helpful errors
    let result = Cli::try_parse_from(["dora", "invalid-command"]);
    assert!(result.is_err());
    // Validate error message suggests valid alternatives
}
```

### Manual Testing Procedures
1. **Existing Command Compatibility**
   ```bash
   # All of these should work exactly as before
   dora up
   dora start dataflow.yml
   dora stop my-dataflow
   dora list
   dora logs node-name
   ```

2. **New Global Flags**
   ```bash
   # Test new global options
   dora --ui-mode cli ps
   dora --output json ps
   dora --no-hints inspect node-name
   dora ps --help  # Should show global flags
   ```

3. **Help and Error Messages**
   ```bash
   # Test help quality
   dora --help
   dora ps --help
   dora invalid-command  # Should suggest alternatives
   ```

## 📚 Resources

### Clap Documentation
- [Clap Derive Tutorial](https://docs.rs/clap/latest/clap/_derive/_tutorial/index.html)
- [Command Line Interface Guidelines](https://clig.dev/)
- [Clap Builder vs Derive](https://docs.rs/clap/latest/clap/_tutorial/index.html)

### Code Examples
```rust
// Example of command trait for consistency
pub trait CommandExecute {
    async fn execute(&self, context: &ExecutionContext) -> Result<()>;
}

// Example of output formatting
impl OutputFormat {
    pub fn render<T: Serialize>(&self, data: &T) -> Result<String> {
        match self {
            OutputFormat::Json => serde_json::to_string_pretty(data),
            OutputFormat::Yaml => serde_yaml::to_string(data),
            OutputFormat::Table => self.render_table(data),
            _ => self.render_compact(data),
        }
    }
}
```

### Backward Compatibility Strategy
- Maintain all existing command names as primary commands
- Add aliases for improved naming (`list` -> `ps`)
- Preserve all existing flag behaviors
- Ensure existing scripts continue to work without modification

## ✅ Definition of Done
- [ ] All existing CLI commands parse and execute correctly
- [ ] New three-tier command structure is implemented
- [ ] Global flags (`--ui-mode`, `--output`, `--no-hints`) work correctly
- [ ] Backward compatibility aliases function properly
- [ ] Help text is comprehensive and includes examples
- [ ] Error messages are actionable and suggest alternatives
- [ ] Unit tests cover all parsing scenarios
- [ ] Integration tests validate end-to-end functionality
- [ ] Manual testing confirms no regression in existing behavior
- [ ] Code review completed by team member
- [ ] Documentation updated to reflect new CLI structure

## 📝 Implementation Notes

### Code Organization
```
src/cli/
├── mod.rs           # Main CLI structure and parsing
├── commands/        # Individual command implementations
│   ├── mod.rs
│   ├── ps.rs
│   ├── start.rs
│   ├── stop.rs
│   └── ...
├── context.rs       # Execution context (for future issues)
├── output.rs        # Output formatting utilities
└── traits.rs       # Common command traits
```

### Migration Strategy
1. **Phase 1**: Implement new structure alongside existing
2. **Phase 2**: Migrate commands one by one
3. **Phase 3**: Remove old command handling
4. **Phase 4**: Add comprehensive testing

This foundational work enables all subsequent CLI enhancements while maintaining full backward compatibility and setting up the architecture for intelligent interface selection.