# Issue #002: Create Execution Context Detection System

## 📋 Summary
Implement a comprehensive execution context detection system that determines the environment in which CLI commands are running. This system will detect TTY status, pipe usage, scripting contexts, terminal capabilities, and user preferences to enable intelligent interface selection.

## 🎯 Objectives
- Detect interactive vs. non-interactive execution contexts
- Identify automation/scripting environments
- Capture terminal capabilities and constraints
- Provide foundation for smart interface selection
- Support user preference integration

**Success Metrics:**
- Accurately detects all major execution contexts (TTY, pipes, scripts, CI)
- Provides reliable terminal capability detection
- Integrates seamlessly with interface selection logic
- Supports both synchronous and asynchronous context detection

## 🛠️ Technical Requirements

### What to Build

#### 1. ExecutionContext Structure
```rust
// src/cli/context.rs
#[derive(Debug, Clone)]
pub struct ExecutionContext {
    /// Whether running in a TTY (interactive terminal)
    pub is_tty: bool,
    
    /// Whether output is being piped to another command
    pub is_piped: bool,
    
    /// Whether running in a scripting context (bash, CI, etc.)
    pub is_scripted: bool,
    
    /// Terminal size if available
    pub terminal_size: Option<(u16, u16)>,
    
    /// Current user preferences
    pub user_preference: UiMode,
    
    /// Detected terminal capabilities
    pub terminal_capabilities: TerminalCapabilities,
    
    /// Environment variables relevant to execution
    pub environment: ExecutionEnvironment,
}

#[derive(Debug, Clone)]
pub struct TerminalCapabilities {
    /// Supports color output
    pub supports_color: bool,
    
    /// Supports Unicode characters
    pub supports_unicode: bool,
    
    /// Supports mouse input
    pub supports_mouse: bool,
    
    /// Terminal type (xterm, screen, etc.)
    pub terminal_type: Option<String>,
}

#[derive(Debug, Clone)]
pub struct ExecutionEnvironment {
    /// CI/CD environment detected
    pub ci_environment: Option<CiEnvironment>,
    
    /// Shell type (bash, zsh, fish, etc.)
    pub shell_type: Option<String>,
    
    /// Environment variables affecting behavior
    pub relevant_env_vars: HashMap<String, String>,
}
```

#### 2. Context Detection Implementation
```rust
impl ExecutionContext {
    /// Create context by detecting current environment
    pub fn detect() -> Self {
        Self {
            is_tty: Self::detect_tty(),
            is_piped: Self::detect_piped(),
            is_scripted: Self::detect_scripted(),
            terminal_size: Self::detect_terminal_size(),
            user_preference: Self::load_user_preference(),
            terminal_capabilities: TerminalCapabilities::detect(),
            environment: ExecutionEnvironment::detect(),
        }
    }
    
    /// Fast synchronous detection for basic contexts
    pub fn detect_basic() -> Self {
        // Minimal detection for performance-critical paths
    }
    
    /// Comprehensive async detection with external tool checks
    pub async fn detect_comprehensive() -> Self {
        // Full detection including external tool availability
    }
}
```

#### 3. Detection Methods
```rust
impl ExecutionContext {
    fn detect_tty() -> bool {
        use std::io::IsTerminal;
        std::io::stdin().is_terminal() && std::io::stdout().is_terminal()
    }
    
    fn detect_piped() -> bool {
        use std::io::IsTerminal;
        !std::io::stdout().is_terminal()
    }
    
    fn detect_scripted() -> bool {
        // Check for CI environments
        if Self::is_ci_environment() {
            return true;
        }
        
        // Check for common scripting indicators
        if env::var("TERM").unwrap_or_default() == "dumb" {
            return true;
        }
        
        // Check parent process name
        if let Ok(parent) = Self::get_parent_process() {
            match parent.as_str() {
                "bash" | "sh" | "zsh" | "fish" | "python" | "node" => true,
                _ => false,
            }
        } else {
            false
        }
    }
    
    fn is_ci_environment() -> bool {
        // Detect common CI environments
        env::var("CI").is_ok() ||
        env::var("GITHUB_ACTIONS").is_ok() ||
        env::var("GITLAB_CI").is_ok() ||
        env::var("JENKINS_URL").is_ok() ||
        env::var("BUILDKITE").is_ok()
    }
}
```

### Why This Approach

**Comprehensive Detection Benefits:**
- Enables intelligent interface selection
- Prevents TUI in inappropriate contexts
- Optimizes for specific environments (CI, pipes, etc.)
- Supports graceful degradation

**Terminal Capability Detection:**
- Ensures appropriate visual features
- Prevents Unicode/color issues
- Enables progressive enhancement
- Supports accessibility needs

**Environment-Aware Behavior:**
- CI-friendly minimal output
- Shell-specific optimizations
- User preference respect
- Context-appropriate defaults

### How to Implement

#### Step 1: Add Dependencies (15 minutes)
```toml
# Add to Cargo.toml
[dependencies]
crossterm = "0.27"           # Terminal detection and capabilities
is-terminal = "0.4"          # TTY detection
sysinfo = "0.29"             # Process information
termsize = "0.1"             # Terminal size detection
```

#### Step 2: Implement Core Detection (3 hours)
1. **Create `src/cli/context.rs`** with main structures
2. **Implement TTY and pipe detection** using is-terminal
3. **Add scripting context detection** with CI environment checks
4. **Implement terminal size detection** with fallback handling

#### Step 3: Add Terminal Capabilities (2 hours)
1. **Detect color support** using environment variables and terminal type
2. **Check Unicode support** based on locale and terminal
3. **Implement mouse support detection** via terminal capabilities
4. **Add terminal type identification** from TERM environment variable

#### Step 4: Environment Detection (2 hours)
1. **Implement CI environment detection** for major platforms
2. **Add shell type detection** from parent process and environment
3. **Collect relevant environment variables** for behavior tuning
4. **Create environment classification** system

#### Step 5: Integration and Testing (1 hour)
1. **Integrate with CLI parsing** from Issue #001
2. **Add performance optimization** for common cases
3. **Implement caching** for expensive detection operations
4. **Create comprehensive test suite**

## 🔗 Dependencies
**Depends On:** Issue #001 (Hybrid Command Framework)
**Blocks:** Issue #003 (Interface Selection Engine)

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_tty_detection() {
        // Mock different TTY states
        let context = ExecutionContext::detect();
        // Validate TTY detection logic
    }
    
    #[test]
    fn test_ci_environment_detection() {
        // Test various CI environment variables
        env::set_var("CI", "true");
        assert!(ExecutionContext::is_ci_environment());
        
        env::set_var("GITHUB_ACTIONS", "true");
        assert!(ExecutionContext::is_ci_environment());
    }
    
    #[test]
    fn test_terminal_capabilities() {
        let caps = TerminalCapabilities::detect();
        // Validate capability detection
    }
    
    #[test]
    fn test_scripted_detection() {
        // Mock various scripting contexts
        env::set_var("TERM", "dumb");
        let context = ExecutionContext::detect();
        assert!(context.is_scripted);
    }
}
```

### Integration Tests
```rust
// tests/context_integration.rs
#[test]
fn test_context_in_different_environments() {
    // Test context detection in various scenarios
    
    // Interactive terminal
    let interactive_context = ExecutionContext::detect();
    
    // Piped output (simulate)
    // CI environment (simulate)
    // Scripted environment (simulate)
}

#[tokio::test]
async fn test_comprehensive_detection() {
    let context = ExecutionContext::detect_comprehensive().await;
    // Validate comprehensive detection includes all fields
}
```

### Manual Testing Procedures
1. **Interactive Terminal Testing**
   ```bash
   # Should detect as interactive TTY
   dora ps
   echo $?  # Should detect context correctly
   ```

2. **Piped Output Testing**
   ```bash
   # Should detect as piped
   dora ps | grep demo
   dora ps > output.txt
   ```

3. **CI Environment Testing**
   ```bash
   # Should detect as scripted/CI
   CI=true dora ps
   GITHUB_ACTIONS=true dora ps
   ```

4. **Terminal Capability Testing**
   ```bash
   # Test color support detection
   TERM=xterm-256color dora ps
   TERM=dumb dora ps
   
   # Test Unicode support
   LC_ALL=C dora ps
   LC_ALL=en_US.UTF-8 dora ps
   ```

## 📚 Resources

### Terminal Detection References
- [is-terminal crate documentation](https://docs.rs/is-terminal/)
- [crossterm terminal capabilities](https://docs.rs/crossterm/latest/crossterm/)
- [ANSI Terminal Guide](https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797)

### CI Environment Variables
- [GitHub Actions](https://docs.github.com/en/actions/learn-github-actions/environment-variables)
- [GitLab CI](https://docs.gitlab.com/ee/ci/variables/predefined_variables.html)
- [Jenkins](https://www.jenkins.io/doc/book/pipeline/jenkinsfile/#using-environment-variables)

### Code Examples
```rust
// Example of context-aware behavior
impl ExecutionContext {
    pub fn should_use_interactive_features(&self) -> bool {
        self.is_tty && 
        !self.is_scripted && 
        self.terminal_capabilities.supports_color
    }
    
    pub fn get_optimal_output_width(&self) -> usize {
        self.terminal_size
            .map(|(width, _)| width as usize)
            .unwrap_or(80)  // Default width
    }
    
    pub fn supports_realtime_updates(&self) -> bool {
        self.is_tty && 
        !self.is_piped && 
        self.terminal_capabilities.supports_unicode
    }
}
```

## ✅ Definition of Done
- [ ] ExecutionContext structure implemented with all required fields
- [ ] TTY detection works correctly in interactive and non-interactive contexts
- [ ] Pipe detection accurately identifies when output is redirected
- [ ] Scripting context detection identifies CI and automation environments
- [ ] Terminal capabilities detection works across major terminal types
- [ ] Environment detection classifies CI platforms and shell types
- [ ] Performance is optimized for common detection scenarios
- [ ] Comprehensive unit tests cover all detection methods
- [ ] Integration tests validate behavior in different environments
- [ ] Manual testing confirms accurate detection across scenarios
- [ ] Documentation includes usage examples and detection logic
- [ ] Code review completed with focus on edge cases
- [ ] Benchmarks confirm detection performance is acceptable

## 📝 Implementation Notes

### Performance Considerations
- Cache expensive operations (process info, file system checks)
- Provide fast path for basic detection
- Lazy load comprehensive detection when needed
- Avoid blocking operations in synchronous detection

### Edge Cases to Handle
- Windows subsystem for Linux (WSL)
- Terminal multiplexers (tmux, screen)
- Remote SSH sessions
- Container environments (Docker, Kubernetes)
- IDE integrated terminals (VS Code, IntelliJ)

### Future Extensions
- Plugin system for custom context detection
- User-defined context rules
- Context history and learning
- Performance profiling integration

This context detection system provides the intelligent foundation needed for the hybrid CLI to make appropriate interface selection decisions while remaining fast and reliable across all execution environments.