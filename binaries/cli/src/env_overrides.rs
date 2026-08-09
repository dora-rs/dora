//! `--env KEY=VALUE` support for `dora start` / `dora run`
//! (dora-rs/dora#2919).
//!
//! With `dora run`, nodes historically inherited the CLI process
//! environment; with `dora start` they inherit the *daemon's*, so a
//! `MYVAR=x dora start …` invocation silently configured nothing. The
//! portable spelling for both commands is `--env`, implemented as a
//! CLI-side merge into the dataflow-level `env:` block of the descriptor
//! that is already shipped to the coordinator/daemon — no protocol
//! change, and the daemon-side env denylist still applies.
//!
//! Precedence (most specific wins):
//! node `env:`  >  `--env`  >  dataflow-level `env:` in the YAML.
//! The node-over-global half is the existing (private) `merge_env`
//! behavior in `dora-core`'s descriptor resolution; this module
//! implements the `--env`-over-global half.
//!
//! `--env` applies at *spawn* time only. Neither `dora start` nor
//! `dora run` re-runs `build:` commands, so build-time environment
//! still comes from node/dataflow `env:` at `dora build` time.

use std::collections::BTreeMap;

use dora_message::descriptor::{Descriptor, EnvValue};
use eyre::{Context, Result, bail};

/// Parse repeated `--env KEY=VALUE` flags. The value may itself contain
/// `=` (only the first one splits). A repeated key keeps the last
/// occurrence, matching `docker run -e` behavior.
///
/// Values are rejected unless they survive the descriptor's wire
/// encoding unchanged — see [`ensure_wire_safe`].
pub fn parse_env_overrides(flags: &[String]) -> Result<BTreeMap<String, EnvValue>> {
    let mut overrides = BTreeMap::new();
    for flag in flags {
        let Some((key, value)) = flag.split_once('=') else {
            bail!("invalid --env `{flag}`: expected KEY=VALUE");
        };
        if key.is_empty() {
            bail!("invalid --env `{flag}`: empty key");
        }
        ensure_wire_safe(key, value)?;
        overrides.insert(key.to_string(), EnvValue::String(value.to_string()));
    }
    Ok(overrides)
}

/// Reject a `--env` value that the descriptor's wire encoding would not
/// deliver verbatim.
///
/// `EnvValue` is an untagged enum whose variants deserialize through
/// `with_expand_envs`, so a value is re-interpreted *every time the
/// descriptor is deserialized* — in the coordinator, then again in the
/// daemon. Two consequences, both verified against `dora-message`:
///
/// * **`$` is expanded in the receiving process.** A wire value of
///   `${DORA_AUTH_TOKEN}` becomes the *coordinator's or daemon's* token
///   by the time a node sees it, which would let `--env` read host
///   secrets that `strip_denied_env` exists to keep out of nodes (that
///   guard matches key names, so an innocuous key sails through).
///   YAML `env:` values are expanded CLI-side when the descriptor is
///   read, so they reach the wire already substituted — `--env` is the
///   only path that can put an unexpanded `$` on it.
/// * **Numeric-looking strings are coerced.** `1.10` arrives as `1.1`,
///   `01234` as `1234`, because the untagged enum tries `Bool`,
///   `Integer` and `Float` before `String`.
///
/// Escaping cannot fix either: the value is deserialized twice, so a
/// `$$` that survives one hop is expanded on the next. Rejecting is the
/// honest option — silently handing a node a different value than the
/// operator typed is worse than refusing to start.
///
/// The `$` rule is deliberately **syntactic**, not "does this value
/// survive a round-trip in the CLI's process". That round-trip is not a
/// sound test of the security property: expansion happens against
/// *whichever* environment decodes the descriptor, so a CLI-local
/// variable that refers to itself (`DORA_AUTH_TOKEN='${DORA_AUTH_TOKEN}'`)
/// makes the CLI-side round-trip an identity — passing the check — while
/// the wire still carries `${DORA_AUTH_TOKEN}` for the daemon to expand
/// against the *real* secret. Refusing every `$` closes that fixed-point
/// bypass by never letting an expandable value reach the wire at all.
fn ensure_wire_safe(key: &str, value: &str) -> Result<()> {
    if value.contains('$') {
        bail!(
            "--env `{key}={value}` contains `$`, which cannot be carried in a \
             dataflow descriptor: the process that receives it (coordinator, \
             then daemon) expands the value against ITS OWN environment, so \
             `$VAR` would resolve to that host's variable rather than \
             yours.\n\n  \
             hint: expand it in your shell first (`--env {key}=\"$VAR\"` \
             without quotes around the `$`), or set this one in the node's \
             `env:` block in the dataflow YAML."
        );
    }

    // Beyond `$`, the untagged enum coerces numeric-looking strings —
    // `1.10` decodes as Float(1.1). A round-trip IS a sound test for
    // that, since no environment lookup is involved.
    let encoded = serde_json::to_string(&EnvValue::String(value.to_string()))
        .with_context(|| format!("failed to encode --env `{key}`"))?;
    match serde_json::from_str::<EnvValue>(&encoded) {
        Ok(decoded) if decoded.to_string() == value => Ok(()),
        Ok(decoded) => {
            bail!(
                "--env `{key}={value}` would not survive the dataflow descriptor's \
                 encoding: nodes would receive `{decoded}` instead.\n\n  \
                 hint: numeric-looking values are coerced (`1.10` -> `1.1`, \
                 `01234` -> `1234`). Set this one in the node's `env:` block in \
                 the dataflow YAML instead."
            );
        }
        Err(err) => {
            bail!(
                "--env `{key}={value}` cannot be represented in the dataflow \
                 descriptor: {err}.\n\n  \
                 hint: set this one in the node's `env:` block in the dataflow \
                 YAML instead."
            );
        }
    }
}

/// Merge `--env` overrides into the descriptor's dataflow-level `env:`
/// block, with the overrides winning on key conflict. Per-node `env:`
/// entries still win over the result — that is `merge_env`'s existing
/// contract in `dora-core`, exercised end to end by the e2e test.
///
/// Callers in `dora start` must apply this AFTER the dataflow-session
/// fingerprint checks: both the hub `fingerprint_source` comparison and
/// `invalidate_if_build_inputs_changed` hash the descriptor (env
/// included), so an earlier merge would reject hub dataflows as
/// "changed since build" and spuriously invalidate the cached build id
/// on every `--env` change.
pub fn apply_env_overrides(descriptor: &mut Descriptor, overrides: BTreeMap<String, EnvValue>) {
    if overrides.is_empty() {
        // Don't turn `env: None` into `Some({})` — a no-op invocation
        // must leave the descriptor byte-identical.
        return;
    }
    descriptor
        .env
        .get_or_insert_with(Default::default)
        .extend(overrides);
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_core::descriptor::DescriptorExt;

    fn flags(items: &[&str]) -> Vec<String> {
        items.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn parses_key_value_pairs() {
        let parsed = parse_env_overrides(&flags(&["FOO=bar", "SEED=42"])).unwrap();
        assert_eq!(parsed["FOO"], EnvValue::String("bar".into()));
        assert_eq!(parsed["SEED"], EnvValue::String("42".into()));
    }

    #[test]
    fn value_may_contain_equals() {
        let parsed = parse_env_overrides(&flags(&["OPTS=--level=debug"])).unwrap();
        assert_eq!(parsed["OPTS"], EnvValue::String("--level=debug".into()));
    }

    #[test]
    fn empty_value_is_allowed() {
        // `--env FOO=` explicitly sets an empty string, mirroring shells.
        let parsed = parse_env_overrides(&flags(&["FOO="])).unwrap();
        assert_eq!(parsed["FOO"], EnvValue::String(String::new()));
    }

    #[test]
    fn missing_equals_is_rejected() {
        let result = parse_env_overrides(&flags(&["JUSTAKEY"]));
        assert!(result.is_err(), "expected Err, got {result:?}");
    }

    #[test]
    fn empty_key_is_rejected() {
        let result = parse_env_overrides(&flags(&["=value"]));
        assert!(result.is_err(), "expected Err, got {result:?}");
    }

    #[test]
    fn repeated_key_keeps_the_last_occurrence() {
        let parsed = parse_env_overrides(&flags(&["FOO=first", "FOO=second"])).unwrap();
        assert_eq!(parsed["FOO"], EnvValue::String("second".into()));
    }

    /// The security property: a `$` that reaches the wire is expanded by
    /// whichever process deserializes the descriptor next (coordinator,
    /// then daemon), so `--env X=${DORA_AUTH_TOKEN}` would hand a node
    /// the DAEMON's token. `strip_denied_env` cannot catch it — that
    /// guard matches key names, and the key here is innocuous.
    #[test]
    fn rejects_values_that_would_be_env_expanded_on_the_wire() {
        for value in [
            "LEAK=${DORA_TEST_2919_UNSET}", // unset here, maybe set on the host
            "LEAK=${PATH}",                 // set here AND on the host
            "LEAK=$PATH",                   // brace-less form
            "LEAK=prefix-${PATH}-suffix",   // embedded, not the whole value
        ] {
            let result = parse_env_overrides(&flags(&[value]));
            let err = result.expect_err("a wire-expandable value must be rejected");
            assert!(
                err.to_string().contains("LEAK"),
                "the error must name the offending key: {err}"
            );
        }
    }

    /// The fixed-point bypass this check exists to close: expansion runs
    /// against whichever environment decodes the descriptor, so a
    /// CLI-local variable that refers to itself makes a CLI-side
    /// round-trip an identity — while the wire still carries `$VAR` for
    /// the DAEMON to expand against the real secret. A round-trip test
    /// passes this; a syntactic `$` rule does not.
    ///
    /// Uses a scoped child process rather than `set_var`, which is
    /// `unsafe` in edition 2024 and would race sibling tests.
    #[test]
    fn self_referential_local_variable_cannot_smuggle_a_dollar_through() {
        let exe = std::env::current_exe().expect("test binary path");
        let output = std::process::Command::new(exe)
            .args(["--exact", "env_overrides::tests::self_referential_child"])
            .env("DORA_TEST_2919_SELFREF", "${DORA_TEST_2919_SELFREF}")
            .env("RUST_TEST_NOCAPTURE", "0")
            .output()
            .expect("failed to re-run test binary");
        assert!(
            output.status.success(),
            "the self-referential value was NOT rejected — a `$` reached the \
             wire.\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr),
        );
    }

    /// Child of `self_referential_local_variable_cannot_smuggle_a_dollar_through`;
    /// only meaningful with `DORA_TEST_2919_SELFREF` set to its own
    /// reference, so it is a no-op when run directly.
    #[test]
    fn self_referential_child() {
        let Ok(value) = std::env::var("DORA_TEST_2919_SELFREF") else {
            return; // not the child invocation
        };
        assert_eq!(value, "${DORA_TEST_2919_SELFREF}", "child env not set up");
        let result = parse_env_overrides(&flags(&["LEAK=${DORA_TEST_2919_SELFREF}"]));
        assert!(
            result.is_err(),
            "a self-referential expansion round-trips to itself, so it must be \
             caught syntactically rather than by comparison, got {result:?}"
        );
    }

    /// A literal `$` cannot be carried either: escaping does not compose
    /// across two deserialization hops, so this is refused rather than
    /// silently mangled.
    #[test]
    fn rejects_values_containing_a_literal_dollar() {
        let result = parse_env_overrides(&flags(&["PW=p$ssw0rd"]));
        assert!(result.is_err(), "expected Err, got {result:?}");
    }

    /// The untagged `EnvValue` enum tries Bool/Integer/Float before
    /// String, so numeric-looking values are coerced in transit. Better
    /// to refuse than to hand a node `1.1` when the operator typed
    /// `1.10`.
    #[test]
    fn rejects_values_the_wire_encoding_would_coerce() {
        for value in ["VERSION=1.10", "ZIP=01234", "SCI=1e5"] {
            let result = parse_env_overrides(&flags(&[value]));
            assert!(
                result.is_err(),
                "`{value}` is silently coerced on the wire and must be rejected, got {result:?}"
            );
        }
    }

    /// Values that DO survive the encoding must still be accepted —
    /// including ones that merely look risky.
    #[test]
    fn accepts_values_that_survive_the_wire_encoding() {
        let parsed = parse_env_overrides(&flags(&[
            "PLAIN=hello world",
            "PATHISH=/usr/local/bin:/usr/bin",
            "UNICODE=café ☕",
            "DASHED=--level=debug",
            "TRUEISH=truthy",
            "EMPTY=",
        ]))
        .expect("these values are wire-safe");
        assert_eq!(parsed["PLAIN"], EnvValue::String("hello world".into()));
        assert_eq!(parsed["UNICODE"], EnvValue::String("café ☕".into()));
        assert_eq!(parsed["EMPTY"], EnvValue::String(String::new()));
    }

    #[test]
    fn no_overrides_leaves_descriptor_untouched() {
        let mut descriptor: Descriptor = Descriptor::parse(b"nodes: []".to_vec()).unwrap();
        assert!(descriptor.env.is_none());
        apply_env_overrides(&mut descriptor, BTreeMap::new());
        assert!(
            descriptor.env.is_none(),
            "a no-op --env must not materialize an empty env block"
        );
    }

    #[test]
    fn overrides_win_over_dataflow_level_env() {
        let yaml = b"
env:
  FROM_YAML: yaml
  SHARED: yaml
nodes: []
"
        .to_vec();
        let mut descriptor: Descriptor = Descriptor::parse(yaml).unwrap();
        let overrides = parse_env_overrides(&flags(&["SHARED=cli", "CLI_ONLY=cli"])).unwrap();
        apply_env_overrides(&mut descriptor, overrides);

        let env = descriptor.env.as_ref().unwrap();
        assert_eq!(env["SHARED"], EnvValue::String("cli".into()));
        assert_eq!(env["CLI_ONLY"], EnvValue::String("cli".into()));
        assert_eq!(
            env["FROM_YAML"],
            EnvValue::String("yaml".into()),
            "non-conflicting yaml keys must survive"
        );
    }

    /// The premise behind the merge PLACEMENT in `dora start`: applying
    /// `--env` changes the descriptor's source fingerprint, which is
    /// compared against `dora build`-time state for hub dataflows and
    /// feeds `invalidate_if_build_inputs_changed`. Hoisting the merge
    /// above those checks would therefore reject hub dataflows as
    /// "changed since build" and clobber the cached build id on every
    /// `--env` run — this pins that the ordering constraint is real, not
    /// just asserted in a comment.
    #[test]
    fn applying_overrides_changes_the_descriptor_fingerprint() {
        let mut descriptor: Descriptor = Descriptor::parse(b"nodes: []".to_vec()).unwrap();
        let before = serde_yaml::to_string(&descriptor).unwrap();
        apply_env_overrides(
            &mut descriptor,
            parse_env_overrides(&flags(&["FOO=bar"])).unwrap(),
        );
        let after = serde_yaml::to_string(&descriptor).unwrap();
        assert_ne!(
            before, after,
            "if --env did not alter the descriptor, the merge-placement \
             constraint in `dora start` would be vacuous"
        );
    }

    /// The full precedence chain, through the same `merge_env` the
    /// daemon-side spawn uses: node `env:` > `--env` > dataflow `env:`.
    #[test]
    fn node_env_still_wins_after_overrides() {
        let yaml = b"
env:
  VAR: yaml
nodes:
  - id: probe
    path: probe.bin
    env:
      VAR: node
    outputs:
      - value
"
        .to_vec();
        let mut descriptor: Descriptor = Descriptor::parse(yaml).unwrap();
        let overrides = parse_env_overrides(&flags(&["VAR=cli"])).unwrap();
        apply_env_overrides(&mut descriptor, overrides);

        let resolved = descriptor.resolve_aliases_and_set_defaults().unwrap();
        let node = &resolved[&"probe".to_string().into()];
        assert_eq!(
            node.env.as_ref().unwrap()["VAR"],
            EnvValue::String("node".into()),
            "node-level env must win over --env"
        );
    }
}
