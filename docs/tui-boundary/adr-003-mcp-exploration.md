# ADR-003: Exploring MCP Alignment for Dora UI Protocol

- **Date:** 2025-03-24
- **Status:** Proposed
- **Related:** ADR-002 (HTTP/SSE protocol), docs/tui-boundary/touchpoints.md

## Context

ADR-002 defined an HTTP+JSON+SSE protocol between Dora’s UIs and the coordinator. This bespoke
contract matches our immediate needs (TUI, legacy CLI shim) but diverges from emerging standards
such as the Model Context Protocol (MCP). MCP provides a JSON-RPC 2.0 based interface for
tool/resource discovery and notifications, with SDKs (including Rust) that could reduce our client
maintenance burden and unlock interoperability with MCP-aware agents.

We want to understand what it would take to expose Dora’s control/data operations via MCP rather
than our custom REST/SSE scheme, and whether the change is worth the added complexity.

## Decision (pending)

Treat this ADR as a spike plan. We will prototype an MCP façade over a subset of Dora operations and
evaluate fit, developer ergonomics, and interoperability benefits before committing to migration.

## Proposed Exploration Steps

1. **Map capabilities:**  
   - Audit current endpoints (ADR-002) and identify how they would surface as MCP resources or
     tools.  
   - Draft a capability namespace (`dora.dataflows.*`, `dora.logs.*`, `dora.telemetry.*`).

2. **Prototype MCP server shim:**  
   - Implement a small adapter that translates MCP JSON-RPC requests into existing coordinator
     calls (can live alongside the protocol gateway).  
   - Support minimal operations first: `list_dataflows`, `stream_logs`, `stream_metrics`.

3. **Adapt client using MCP Rust SDK:**  
   - Replace the TUI’s HTTP protocol client (behind a feature flag) with the MCP SDK, mapping tool
     invocations and resource subscriptions to the existing service traits.  
   - Evaluate how streaming (notifications) integrates versus the current SSE cache.

4. **Compare ergonomics & performance:**  
   - Measure latency/throughput for telemetry/log streams.  
   - Assess code size/complexity on both client and server halves.  
   - Document gaps (auth, versioning, schema evolution) that MCP handles vs. what remains custom.

5. **Publish findings:**  
   - If MCP proves advantageous, author a follow-up ADR detailing migration strategy and
     compatibility story (e.g., maintain REST/SSE for existing clients, MCP for new ones).  
   - Otherwise, capture reasons to remain with the current design.

## Considerations

- MCP assumes JSON-RPC framing and capability negotiation. Dora would need to proxy stream updates
  as notifications rather than SSE events.  
- Authentication is not built into MCP; we would still layer our own security model.  
- Existing tooling (TUI, future GUI) must adopt MCP semantics—this may require significant code
  changes compared to thin protocol-client wrappers.  
- Interoperability upside: MCP-compliant tools or LLM agents could control Dora without bespoke
  integrations.

## Status & Next Steps

- No implementation started; ADR captures the investigation plan.  
- Action item: spin up the prototype MCP server in a feature branch, report findings, then decide on
  adopting or shelving MCP alignment.

