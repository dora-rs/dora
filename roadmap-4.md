# Actionable Development Roadmap for DORA

This critically enhanced roadmap outlines a comprehensive development plan for the DORA framework. It integrates features from official documentation with a critical analysis of potential gaps in security, testing, state management, and package management. The roadmap is structured into three logical phases designed to mature the platform for developers, advanced robotics applications, and enterprise-grade deployment.

---

## Phase 1: Foundational Stability & Developer Experience

**Goal:** Address the most significant sources of friction for new and existing users. Stabilize experimental features and fill critical gaps in security and testing to build a trustworthy foundation.

### **Theme 1: Core Framework & Communication**

-   **Stabilize ROS2 Bridge:**
    -   **Action:** Move the existing unstable ROS2 bridge to a production-ready state.
    -   **Details:** Expand capabilities to include full, reliable support for ROS2 **services** and **actions**. This requires addressing the fundamental difference between DORA's dataflow (publish/subscribe) and ROS2's request/response patterns.
        -   **Why Current Lack:** DORA's core is a dataflow engine based on message passing (topics). ROS2 services are synchronous request/response, and actions are asynchronous goal/feedback/result, both requiring state management and a different communication pattern than simple topics.
        -   **What Needs to be Done (High-Level):** Introduce dedicated bridge components within DORA that act as intermediaries, translating between ROS2's service/action semantics and DORA's dataflow model. This includes a ROS2 Service Bridge, a ROS2 Action Bridge, and robust type conversion mechanisms.
        -   **How to Implement (Detailed Plan):**
            1.  **Analyze Existing ROS2 Bridge:** Understand current topic handling and message type management.
            2.  **Define DORA-ROS2 Service/Action API:** Design new `dora-node-api` functions for interacting with services and actions.
            3.  **Implement ROS2 Service Bridge Components:** Create `dora-ros2-service-server` and `dora-ros2-service-client` nodes/operators to handle request/response translation.
            4.  **Implement ROS2 Action Bridge Components:** Develop `dora-ros2-action-server` and `dora-ros2-action-client` nodes/operators to manage the more complex asynchronous goal/feedback/result state machine.
            5.  **Develop Robust Message Type Conversion:** Create a reliable mechanism to map ROS2 `.srv` and `.action` definitions to DORA's Apache Arrow schema.
            6.  **Comprehensive Integration Testing:** Add a comprehensive integration test suite that validates message passing, services, and actions end-to-end with a live ROS2 instance.
-   **Enhance Data Logging & Replay:**
    -   **Action:** Build explicit, indexed data replay functionality.
    -   **Details:** This enhancement is crucial for debugging, testing, and data-driven development in robotics.
        -   **Why it's Critical:**
            1.  **Debugging & Root Cause Analysis:** Replaying exact scenarios helps pinpoint intermittent bugs.
            2.  **Regression Testing:** Ensures new code doesn't break existing functionality by replaying known good data.
            3.  **Data-Driven Development:** Allows iterating on algorithms using real-world data without physical hardware.
            4.  **Simulation & Scenario Testing:** Enables testing complex or dangerous scenarios in a controlled environment.
            5.  **Compliance & Auditing:** Provides a verifiable record of system behavior for critical applications.
        -   **What Needs to be Done (High-Level):**
            1.  **Comprehensive Recording (`dora-record`):** Capture all relevant data (messages, metadata, timestamps, dataflow configuration) efficiently.
            2.  **Intelligent Indexing:** An accompanying index (e.g., SQLite) for quick, flexible lookup and filtering.
            3.  **Accurate Replay (`dora-replay`):** Precisely reproduce recorded data streams, respecting original timings, with speed and filtering control.
            4.  **Metadata Enrichment:** Capture contextual information about the recording session and dataflow.
        -   **How to Implement (Detailed Plan):**
            1.  **Refine `dora-record` Node:** Enhance data capture (timestamps, topic names, source nodes, dataflow config, system events). Implement robust SQLite index generation (pointers to Parquet, message metadata, session metadata, custom tags).
            2.  **Develop `dora-replay` Node:** Implement flexible index reading, advanced filtering (topics, time windows, dataflow UUID), precise timing control (real-time, speed-up/slow-down, step-by-step), and accurate message publishing.
            3.  **CLI Integration & Management:** Provide intuitive CLI commands for managing recordings (start, stop, list, replay, info).
            4.  **Comprehensive Testing:** Develop a robust test suite for `dora-record` and `dora-replay` covering data integrity, timing accuracy, filtering, and stress tests.

### **Theme 2: Developer Experience (DevEx) & Tooling**

-   **Comprehensive CLI Overhaul:**
    -   **Action:** Rework the `dora-cli` from a basic tool to a sophisticated developer utility.
    -   **Details:** Rework the `dora-cli` to provide a Docker-like user experience, focusing on consistency, discoverability, and power.
        -   **Core Principles:**
            *   **Consistent Command Structure:** `dora <command> <subcommand> [options] [arguments]`.
            *   **Clear & Intuitive Naming:** Commands directly reflect action or resource.
            *   **Discoverability:** Comprehensive `help` messages and robust tab completion.
            *   **Output Formatting:** Human-readable by default (tables), machine-readable with flags (`--json`, `----yaml`).
            *   **Actionable Error Messages:** Clear guidance on how to resolve issues.
        -   **Command Refinements:**
            *   **`dora check`:** Retain as a diagnostic/pre-flight command for DORA environment health (e.g., `dora check daemon`, `dora check dependencies`).
            *   **`dora ps`:** Introduce to list running dataflows and their high-level status (ID, Name, Status, Uptime). Replaces the original `dora status` concept.
            *   **`dora stats`:** Introduce to provide live resource usage statistics for running dataflows and nodes (CPU, memory, network I/O).
            *   **`dora inspect <resource_id>`:** Introduce for detailed low-level information about any DORA resource (dataflow, node, recording).
        -   **Enhanced Data Logging & Replay CLI:**
            *   **`dora record` subcommands:** `start`, `stop`, `ls`, `rm`, `inspect` with comprehensive options for filtering, naming, and output.
            *   **`dora replay` subcommands:** `replay` with options for topics, time windows, speed control, looping, and output.
        -   **Other Key Commands:**
            *   **`dora run`:** Enhance with `indicatif` progress bars.
            *   **`dora logs`:** Enhance with `--follow`, `--timestamps`, `--tail` options.
            *   **`dora rm` / `dora stop` / `dora start`:** For managing dataflow lifecycle.
            *   **`dora build`:** Enhance with `indicatif` progress bars.
            *   **`dora system info` / `dora system prune`:** For system-wide management.
            *   **`dora version` / `dora config`:** For CLI and global settings.
        -   **DevEx Tooling:**
            *   **Interactive `dora init` wizard:** Use `inquire` for simplified project creation.
            *   **Progress Bars:** Integrate `indicatif` for `dora run`, `dora build`, `dora record start`, `dora replay`.
-   **Documentation Revitalization:**
    -   **Action:** Create a comprehensive `mdbook` for DORA.
    -   **Details:** The book must include a "Getting Started" guide for new users, deep dives on core concepts (daemon, coordinator, nodes vs. operators), and visual diagrams using `mermaid.js`.

### **Theme 3: Security & Configuration Foundations (New)**

-   **Critique:** The framework currently lacks a clear security model and secret management strategy.
-   **Actionable Items:**
    This theme focuses on laying a secure foundation to ensure data integrity, confidentiality, and availability, and to future-proof the framework for critical applications.
    1.  **Introduce Node Authentication:**
        -   **Why:** To verify the identity of nodes connecting to the daemon/coordinator, preventing unauthorized access.
        -   **What:** A token-based authentication system.
        -   **How:** The `dora-coordinator` generates unique, short-lived tokens for authorized nodes. Nodes present these tokens to the `dora-daemon` for validation during connection handshake. Consider using JWTs for signed tokens.
    2.  **Implement Basic Secret Management:**
        -   **Why:** To securely handle sensitive configuration data (e.g., API keys, passwords) without hardcoding them, reducing exposure risks.
        -   **What:** Support for loading secrets from environment variables and `.env` files.
        -   **How:** DORA components read sensitive values from predefined environment variables. For local development, a `.env` file can be used. Ensure secrets are not logged or exposed in plain text.
    3.  **Add Transport Encryption (Local):**
        -   **Why:** To protect data in transit over local communication channels (e.g., shared memory, local sockets) from inspection or eavesdropping by local attackers.
        -   **What:** Encryption for all local inter-process communication (IPC).
        -   **How:** For local TCP/Unix sockets, implement TLS using a Rust library like `rustls`. For shared memory, design a mechanism to encrypt/decrypt data before/after writing/reading, carefully balancing security with performance.

### **Theme 4: Testing & Simulation Infrastructure (New)**

-   **Critique:** The lack of a formal testing strategy beyond unit tests is a major gap for a robotics framework.
-   **Actionable Items:**
    This theme aims to provide developers with the **visibility, control, and isolation** needed to effectively test and debug DORA dataflows, especially given their declarative nature.
    1.  **Enhanced Unit Testing for Nodes/Operators:**
        -   **Why:** To enable robust, isolated testing of individual node/operator code logic.
        -   **What:** A comprehensive `dora-test-utils` library.
        -   **How:** Provide mock implementations for DORA APIs (`send`, `receive`, `subscribe`, state management) to allow standard unit testing frameworks (e.g., `pytest`, `cargo test`) to test node logic without a running DORA runtime. Include test data generation and assertion helpers.
    2.  **Robust Integration Testing for Dataflows:**
        -   **Why:** To ensure correct interaction between multiple nodes as defined in a YAML dataflow.
        -   **What:** Formalized scenario-based testing and programmatic dataflow control.
        -   **How:**
            *   **Scenario-Based Testing:** Leverage the enhanced `dora-record`/`dora-replay` workflow as the primary method for integration and regression testing.
            *   **Programmatic Control API:** Provide Python and Rust SDKs to programmatically load, start, stop, pause, resume dataflows; inject data onto topics; observe output data; and inspect node/coordinator state from test scripts.
            *   **"Test Harness" Nodes:** Develop specialized nodes like `dora-test-source` (for emitting test data), `dora-test-sink` (for capturing output for assertions), and `dora-assertion-node` (for in-dataflow assertions).
    3.  **Powerful Debugging Tools for Declarative Dataflows:**
        -   **Why:** To provide crucial visibility and control into the runtime behavior of declarative dataflows, which are challenging to debug with traditional methods.
        -   **What:** A suite of CLI and in-dataflow debugging tools.
        -   **How:**
            *   **Enhanced `dora logs`:** Implement advanced filtering, following, and tailing capabilities.
            *   **`dora inspect`:** Provide detailed runtime configuration and state of dataflows and nodes.
            *   **"Probe" Nodes:** Develop generic nodes that can be inserted into any dataflow for debugging, such as `dora-print-node` (for stdout), `dora-save-node` (for file logging), `dora-throttle-node` (for rate control), and `dora-breakpoint-node` (to pause dataflow execution based on conditions).
            *   **Interactive Dataflow Debugger (Ambitious, Future):** Explore a GUI or CLI interface for visualizing dataflow, setting breakpoints, inspecting/injecting messages, and stepping through execution.

---

## Phase 2: Advanced Features & Scalability

**Goal:** Build the complex features required for production applications, with a strong focus on state persistence, security, and robust dependency management.

### **Theme 1: Advanced Runtime & Control Plane**

-   **Migrate Coordinator/Daemon Communication to gRPC:**
    -   **Action:** Replace the custom TCP-based JSON messaging with a robust gRPC framework.
    -   **Details:** Define formal Protobuf services for all coordinator-daemon and CLI-coordinator interactions. Use `tonic` to implement the gRPC servers and clients, providing a strongly-typed, efficient, and more maintainable communication layer.
-   **Implement Advanced Scheduling:**
    -   **Action:** Introduce more complex runtime and scheduling features.
    -   **Details:** This involves leveraging underlying operating system capabilities to ensure predictable and timely execution of DORA tasks, crucial for robotics and mission-critical applications.
        -   **Why it's Important:**
            1.  **Real-time Guarantees:** Meet strict deadlines for control loops and sensor processing.
            2.  **Predictability & Reduced Jitter:** Ensure tasks execute within predictable timeframes, minimizing latency variations.
            3.  **Resource Management & Isolation:** Precisely allocate CPU, memory, and I/O resources to critical tasks.
            4.  **Mixed-Criticality Systems:** Run tasks with different criticality levels on the same platform without interference.
            5.  **Optimized Performance:** Improve efficiency by reducing context switching and enhancing cache locality.
            6.  **Sub-dataflow Graphs:** Enable modularity and dynamic management of dataflow parts for granular scheduling control.
        -   **What Needs to be Done (High-Level):**
            1.  **Time-aware/Deadline-based Scheduling:** Define and enforce execution deadlines for operators/nodes.
            2.  **Resource Prioritization & Isolation:** Assign higher priority and isolate resources for critical tasks.
            3.  **Sub-dataflow Management:** Runtime support for dynamically managing nested dataflows.
            4.  **Integration with OS Scheduling Primitives:** Leverage Linux real-time features and cgroups.
        -   **How to Implement (Detailed Plan):**
            1.  **Define Scheduling Requirements & Metrics:** Identify typical deadlines and define metrics like latency, jitter, CPU utilization, and deadline misses.
            2.  **Extend Dataflow YAML for Scheduling Directives:** Allow specifying `scheduling_policy` (e.g., `realtime_fifo`), `priority`, `cpu_affinity`, `deadline_ms`, `memory_limit_mb`, and `cpu_shares` for nodes/operators.
            3.  **`dora-runtime` Enhancements:**
                *   **OS Scheduler Integration:** Use `libc`/`nix` (Rust) or `os` (Python) to call `sched_setscheduler`, `setpriority`, `sched_setaffinity`. (Note: Requires `CAP_SYS_NICE` or root).
                *   **Resource Cgroups Integration (Linux):** `dora-daemon` manages cgroups for nodes to control CPU, memory, I/O.
                *   **Deadline Monitoring & Reporting:** Monitor and report deadline misses as critical events.
                *   **Sub-dataflow Management:** Extend runtime to understand and manage nested dataflow graphs for dynamic loading/unloading.
            4.  **Coordinator Role in Scheduling:** Coordinator parses YAML directives and instructs Daemons to apply them.
            5.  **CLI Integration:** `dora run` accepts scheduling parameters; `dora ps`/`inspect` show applied policies; `dora stats` could show deadline adherence.
            6.  **Testing & Validation:** Develop benchmarks to measure latency, jitter, and deadline adherence under load, testing on various Linux kernel configurations.

### **Theme 2: State Management & Persistence (New)**

-   **Critique:** Dora is primarily stateless, which limits its applicability for many robotics tasks like SLAM.
-   **Actionable Items:**
    1.  **Implement a Distributed State Store for Coordinator State:**
        -   **Action:** Integrate a fault-tolerant, highly-available, and consistent state backend for the `dora-coordinator`.
        -   **Details:** This will eliminate the coordinator as a single point of failure for state. Two primary options are considered:
            -   **Option A: `dqlite` (Pragmatic Choice):** Integrate `dqlite` (Distributed SQLite) via its C FFI. The `dora-coordinator` instances will form a `dqlite` cluster, replicating state among themselves using Raft consensus. This offers a battle-tested solution with minimal development effort.
            -   **Option B: `openraft` (Pure Rust Alternative):** Implement a custom distributed state store using `openraft` (a pure-Rust Raft implementation) combined with an embedded key-value store like `sled`. This provides a fully Rust-native solution, offering maximum safety and long-term maintainability, but requires more initial development effort.
    2.  **Refactor Coordinator to be a Stateless Reconciler:**
        -   **Action:** Redesign the `dora-coordinator` to be stateless, relying entirely on the `dqlite` cluster for all persistent state.
        -   **Details:** The coordinator will watch the `dqlite` store for desired state changes (e.g., new dataflow deployments) and reconcile them with the actual state reported by daemons, ensuring the system converges to the desired configuration.
    3.  **Define and Expose State Schema:**
        -   **Action:** Formalize the schema for all control plane state (dataflow definitions, runtime status, daemon health, etc.) stored in `dqlite`.
        -   **Details:** Provide clear APIs for nodes and other components to interact with this state (e.g., `node.get_state()`, `node.set_state()` for application-level state, and coordinator APIs for system state).

### **Theme 3: Hardened Security & Configuration**

-   **Critique:** Foundational security is insufficient for distributed systems.
-   **Actionable Items:**
    1.  **Enforce mTLS for Remote Communication:** As part of the gRPC migration, enforce mutual TLS (mTLS) for all remote communication. The coordinator will act as a Certificate Authority (CA).
    2.  **Introduce Environment-Specific Configuration:** Support layered YAML configurations (e.g., `dora.base.yml`, `dora.production.yml`) to manage different deployment environments cleanly.

### **Theme 4: Package & Dependency Management (New)**

-   **Critique:** The `node-hub` will not scale as a simple folder and will lead to dependency conflicts.
-   **Actionable Items:**
    This theme addresses the critical need for robust package and dependency management to foster a thriving, multi-language DORA ecosystem, ensuring reproducibility, discoverability, and security.
    1.  **Formalize DORA Node Package Specification:**
        -   **Why:** To standardize how DORA nodes are described, regardless of language.
        -   **What:** A formal, language-agnostic specification (e.g., `Dora.toml` or `dora-node.yaml`).
        -   **How:** Define fields for `name`, `version`, `language`, `entrypoint`, `dependencies` (with version constraints), `dora_api_version`, `capabilities`, and `checksum`.
    2.  **Centralized Node Registry:**
        -   **Why:** To provide a single, versioned repository for publishing and discovering DORA nodes.
        -   **What:** An HTTP/HTTPS server storing node metadata and packaged artifacts.
        -   **How:** Implement authentication for publishing, search/discovery API for `dora-cli`, and enforce semantic versioning.
    3.  **Enhanced `dora build` and `dora run` for Dependency Resolution:**
        -   **Why:** To enable seamless fetching, resolution, and management of node dependencies from the registry.
        -   **What:** Modified `dora-cli` commands.
        -   **How:**
            *   **`dora publish <path_to_node_source>`:** Package and publish nodes.
            *   **`dora install <node_name>:<version>`:** Fetch and cache specific node versions.
            *   **`dora build <dataflow.yml>`:** Parse dataflow, resolve dependencies (local cache then registry), compile (Rust/C/C++), prepare isolated environments (Python), and generate a lock file (`dora-lock.yml`) for reproducibility.
            *   **`dora run <dataflow.yml>`:** Use `dora-lock.yml` to ensure exact versions.
    4.  **Implement Runtime Isolation for Multi-Language Nodes:**
        -   **Why:** To prevent dependency conflicts by ensuring each node runs in an isolated environment.
        -   **What:** Language-specific isolation mechanisms.
        -   **How:**
            *   **Python Nodes:** `dora-daemon` creates a dedicated `venv` for each Python node instance and installs dependencies into it.
            *   **Rust/C/C++ Nodes:** Ensure proper RPATH/LD_LIBRARY_PATH handling for dynamic linking. Explore supporting containerization (e.g., Docker) for ultimate isolation and reproducibility, especially for complex system dependencies.
    5.  **Security & Integrity:**
        -   **Why:** To ensure the trustworthiness and safety of published nodes.
        -   **What:** Mechanisms for verifying package integrity and security.
        -   **How:** Require cryptographic signatures for published packages, implement vulnerability scanning, and provide a mechanism for reporting/revoking malicious or vulnerable packages.

### **Theme 5: Safety & Correctness**

-   **Critique:** The prevalence of `unsafe` code in core APIs undermines Rust's safety guarantees.
-   **Actionable Items:**
    This theme aims to enhance the reliability and robustness of DORA's core by addressing the implications of `unsafe` code in Rust and improving Foreign Function Interface (FFI) safety.
    1.  **Refactor Core `unsafe` Blocks:**
        -   **Why:** To uphold Rust's memory and thread safety guarantees, especially in safety-critical robotics applications.
        -   **What:** Systematic review, audit, and refactoring of existing `unsafe` code.
        -   **How:** Identify `unsafe` blocks using tooling, encapsulate raw `unsafe` operations into small, isolated, and thoroughly documented "safe" abstractions. For every remaining `unsafe` block, provide clear "safety proofs" in comments.
    2.  **Adopt `safer-ffi`:**
        -   **Why:** To mitigate risks associated with FFI (interfacing with C/C++ code), which is inherently `unsafe` in Rust.
        -   **What:** Integration of the `safer-ffi` crate.
        -   **How:** Use `safer-ffi` to generate C API bindings that include compile-time and runtime safety checks, reducing common FFI pitfalls. Replace manual `extern "C"` blocks with `safer-ffi` generated code where applicable.
    3.  **Enforce `forbid(unsafe_code)`:**
        -   **Why:** To prevent accidental or careless introduction of new `unsafe` code into critical APIs.
        -   **What:** Compile-time enforcement of `unsafe` code usage.
        -   **How:** Add the `#[forbid(unsafe_code)]` attribute to key API crates (`dora-node-api`, `dora-operator-api`, and potentially `dora-core` after refactoring). This makes the Rust compiler emit an error if any `unsafe` block is found within the annotated crate.

---

## Phase 3: Enterprise-Ready Ecosystem & Cloud Integration

**Goal:** Solidify Dora as a platform for building large-scale, commercial, and mission-critical systems.

### **Theme 1: API-First Architecture & Unified Web Platform**

-   **Critique:** To build a truly modern and accessible ecosystem, DORA needs to move beyond a CLI-only interface to an API-first model that supports a comprehensive, integrated web platform for both developers and operators.
-   **Actionable Items:**
    1.  **Expose a Public Control Plane API:**
        -   **Action:** Formally define, document, and expose the coordinator's gRPC services as a public, stable API.
        -   **Details:** This API is the **foundation** for all other tooling. It will be a versioned, secure, gRPC-based API with a gRPC-Web proxy for browser access. We will publish official client SDKs (Python, JS/TS) to enable a rich third-party ecosystem.
    2.  **Develop a Unified Web Platform for Development & Operations:**
        -   **Action:** Create a single, web-based platform that seamlessly integrates a visual dataflow editor with an interactive operations console.
        -   **Details:**
            -   **Why:** To provide a single pane of glass for the entire application lifecycle. This breaks down the silos between development and operations, allowing for faster iteration, easier debugging, and a more holistic understanding of the system.
            -   **What:** A single web application with two primary, tightly integrated modes:
                *   **A "Design" View:** A drag-and-drop canvas for visually building, configuring, and saving DORA dataflows.
                *   **An "Operate" View:** An interactive console for deploying, monitoring, and controlling running dataflows and fleets. This includes live topic inspection, control over node lifecycles, and visualization of DORA's existing OpenTelemetry tracing data.
            -   **How:** The platform will be a modern frontend application built entirely on the **Public Control Plane API**. Development can be staged, starting with the API, then the "Operate" view for immediate operational value, followed by the "Design" view.

### **Theme 2: Cloud-Native Integration & Fleet Operations**

-   **Implement Fleet Management:**
    -   **Action:** Build tools for orchestrating dataflows across large fleets.
    -   **Details:** Capabilities to deploy, monitor, update, and manage DORA dataflows and components across a large number of distributed `dora-daemon` instances (a "fleet" of robots or edge devices).
        -   **Why:**
            *   **Scalability:** Automates deployment and updates for large robot fleets.
            *   **Centralized Control:** Single pane of glass for orchestrating fleet operations.
            *   **Reliability & Consistency:** Ensures consistent deployments and simplifies recovery.
            *   **Remote Operations:** Enables managing robots in remote or inaccessible locations.
            *   **NAT Traversal:** Crucial for managing devices behind Network Address Translators.
        -   **How:**
            1.  **Daemon Registration & Heartbeats:** Enhance `dora-daemon` to securely register with `dora-coordinator` and send regular heartbeats, providing status and resources.
            2.  **Secure Remote Communication Channel:** Establish a robust, secure, and efficient channel (e.g., Zenoh) between coordinator and remote daemons, secured with mTLS and strong authentication.
            3.  **Deployment Orchestration:** Coordinator gains ability to deploy dataflows to specific daemons/groups based on criteria, supporting phased rollouts and rollbacks.
            4.  **Remote Monitoring & Logging:** Collect aggregated logs, metrics, and events from remote daemons/nodes, feeding into the Web-Based Dashboard.
            5.  **Over-the-Air (OTA) Updates:** Implement a mechanism for securely updating DORA daemon software and node binaries on remote devices, including integrity verification and atomic updates.

### **Theme 3: Framework Maturation & Governance**

-   **Critique:** The project needs formal processes to manage its growth and ensure long-term stability.
-   **Actionable Items:**
    1.  **Establish a Formal RFC Process:** Create a public process for proposing and debating major architectural changes.
        -   **Why:** To provide a transparent, structured, and collaborative process for making significant architectural decisions, preventing chaos and building a historical record. It ensures all stakeholders can contribute to and understand the evolution of the framework.
        -   **What:** A formal system for proposing, debating, and approving major changes via public, version-controlled documents (RFCs).
        -   **How:** Create a dedicated repository or directory for RFCs, define a clear template for proposals, and document the lifecycle of an RFC from draft pull request to final approval by a core team.
    2.  **Define API Stability & Versioning Policy:** Publish a clear commitment to Semantic Versioning and define a roadmap towards a `1.0` release with a Long-Term Support (LTS) policy.
        -   **Why:** To provide predictability and build trust with users, especially those in production or enterprise environments. It clarifies when breaking changes will occur and guarantees long-term support for stable releases.
        -   **What:** A published policy committing to Semantic Versioning (`MAJOR.MINOR.PATCH`) and defining a Long-Term Support (LTS) plan for specific versions.
        -   **How:** Publish a `VERSIONING.md` file that details the commitment to SemVer, defines what constitutes a breaking change for all public APIs, outlines the roadmap to a `1.0` release, and specifies the duration and scope of the LTS policy.

### **Theme 4: Advanced Security**

-   **Critique:** Enterprise applications require more robust secrets management than environment variables.
-   **Actionable Items:**
    1.  **Integrate with External Secret Managers:** Add support for fetching secrets at runtime from systems like HashiCorp Vault or cloud-native services (e.g., AWS/GCP Secret Manager).
        -   **Why:** To move beyond the limitations of environment variables, providing centralized control, robust auditing, fine-grained access policies, and the ability to use dynamic, short-lived secrets, which are critical for high-security environments.
        -   **What:** A framework that allows DORA nodes to fetch secrets directly from specialized secret managers like HashiCorp Vault, AWS Secrets Manager, or Google Cloud Secret Manager.
        -   **How:** Design a pluggable `SecretProvider` interface in the node API. Implement concrete providers for different backends (e.g., Vault, AWS). Extend the dataflow YAML to let users specify the provider and secret mappings. The `dora-runtime` will then securely fetch and inject these secrets into the node's environment at startup, authenticating via the host machine's identity (e.g., IAM role).

---

## Phase 4: AI-Centric Robotics & Intelligence

**Goal:** Position DORA as the premier framework for building and deploying high-performance, AI-driven robotic systems by providing first-class support for the entire ML lifecycle.

### **Theme 1: First-Class AI Compute & Data Path**

-   **Action:** Implement GPU-aware scheduling and zero-copy data pathways.
-   **Details:** To run modern AI models efficiently, DORA must treat GPU resources as a primary concern and minimize data movement overhead.
    -   **Why:** AI workloads are compute-intensive and data-hungry. Without explicit GPU management, nodes will compete for resources, leading to unpredictable performance. Copying large tensors (images, point clouds) between processes creates significant latency.
    -   **What:**
        1.  **GPU-Aware Scheduling:** Allow users to request specific GPU resources (device ID, memory) for nodes.
        2.  **Zero-Copy GPU Communication:** Enable passing data between nodes via CUDA IPC handles or similar mechanisms, allowing a downstream node to access a tensor in GPU memory without it ever being copied.
    -   **How:** Extend the YAML specification for GPU requests. The `dora-daemon` will use libraries like NVML to manage GPU allocation and set environment variables (`CUDA_VISIBLE_DEVICES`). The communication layer will be enhanced to support passing GPU memory handles instead of raw data.

### **Theme 2: Integrated Model Lifecycle Management**

-   **Action:** Integrate a model registry and management tooling into the DORA ecosystem.
-   **Details:** Treat AI models as versioned, discoverable, and deployable assets, just like code.
    -   **Why:** Managing the lifecycle of numerous AI models (different versions, A/B tests, etc.) is a major challenge in production robotics. A simple file-based approach does not scale.
    -   **What:** A system for managing the entire lifecycle of AI models within DORA.
    -   **How:**
        1.  **Model Registry Integration:** Allow `dora.yml` to reference models from a registry (e.g., Hugging Face Hub, or a private registry) with versioning, like `model: my-org/my-model:v2`.
        2.  **`dora-daemon` as Fetcher:** The daemon becomes responsible for downloading the specified model weights and making them available to the node.
        3.  **Dynamic Updates:** (Ambitious) Build mechanisms to update a model in a running dataflow without downtime. This would be a powerful feature for continuous learning and deployment.

### **Theme 3: AI Developer Experience & Tooling**

-   **Action:** Provide a rich set of pre-built nodes and tools tailored for AI development.
-   **Details:** Accelerate the development of AI applications by providing ready-made components and insights.
    -   **Why:** Developers shouldn't have to reinvent the wheel for common AI tasks like inference or data preprocessing.
    -   **What:** A suite of AI-focused tools and nodes.
    -   **How:**
        1.  **Optimized Inference Nodes:** Create and maintain nodes for popular inference runtimes (e.g., `dora-onnx`, `dora-tensorrt`, `dora-tflite`).
        2.  **Data-Centric Tooling:** Enhance `dora-record` and `dora-replay` to facilitate the creation of high-quality datasets for model training and fine-tuning from real-world robot data.
        3.  **Performance Tooling:** Enhance `dora stats` and `dora inspect` to provide detailed AI-specific metrics, such as inference latency, GPU utilization, and memory usage per node.
