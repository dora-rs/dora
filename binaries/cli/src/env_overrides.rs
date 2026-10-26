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

use dora_message::descriptor::{Descriptor, EnvValue, Node};
use eyre::{Result, bail};

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
/// A string `EnvValue` is passed through `with_expand_envs`, so a value is
/// re-interpreted *every time the descriptor is deserialized* — in the
/// coordinator, then again in the daemon. Two consequences:
///
/// * **`$` is expanded in the receiving process.** A wire value of
///   `${DORA_AUTH_TOKEN}` becomes the *coordinator's or daemon's* token
///   by the time a node sees it, which would let `--env` read host
///   secrets that `strip_denied_env` exists to keep out of nodes (that
///   guard matches key names, so an innocuous key sails through).
///   YAML `env:` values are expanded CLI-side when the descriptor is
///   read, but a CLI variable whose *value* holds a `$` still puts one on
///   the wire; [`ensure_descriptor_env_wire_safe`] covers that path.
/// * **Numeric-looking strings are coerced by released 1.x peers.**
///   Coordinators and daemons before the lossless `EnvValue` fix decode
///   `1.10` as `1.1` and `01234` as `1234`, and a newer CLI still talks to
///   them (`versions_compatible` is caret-based). This build no longer
///   coerces, so the check below emulates the old decoding instead of
///   round-tripping through this build's `EnvValue`.
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

    // A pre-fix peer decodes `inf`/`NaN` as a float, which JSON can only
    // re-encode as `null`, so the next hop fails to read the descriptor.
    if value.parse::<f64>().is_ok_and(|f| !f.is_finite()) {
        bail!(
            "--env `{key}={value}` cannot be carried in a dataflow descriptor: \
             older coordinators and daemons read it as a non-finite number, \
             which JSON cannot encode.\n\n  \
             hint: set this one in the node's `env:` block in the dataflow \
             YAML instead."
        );
    }

    // Beyond `$`, refuse what a pre-fix peer would coerce.
    if let Some(coerced) = legacy_coercion(value) {
        bail!(
            "--env `{key}={value}` would not survive the dataflow descriptor's \
             encoding: nodes would receive `{coerced}` instead.\n\n  \
             hint: numeric-looking values are coerced (`1.10` -> `1.1`, \
             `01234` -> `1234`). Set this one in the node's `env:` block in \
             the dataflow YAML instead."
        );
    }
    Ok(())
}

/// Reject a descriptor whose `env:` values would be expanded again on
/// the coordinator or daemon host (dora-rs/dora#3947).
///
/// YAML `env:` strings are `$`-expanded against the CLI's environment
/// when the descriptor is read, and again by every process that decodes
/// it afterwards. A value is only delivered as read if that second
/// expansion leaves it alone, which fails when the first expansion
/// produced a variable reference: `X: ${FOO}` with
/// `FOO='${DORA_AUTH_TOKEN}'` would hand the node the daemon's token,
/// and `${DB_PASSWORD}` holding `p$ssw0rd` would reach the node as
/// `p` plus the host's `$ssw0rd`, or fail to decode on the coordinator.
///
/// The check is syntactic, like [`ensure_wire_safe`]'s, and runs on the
/// already-expanded values: any `$` that `shellexpand` would interpret
/// (see [`has_expandable_dollar`]) is refused. A `$` that it keeps
/// verbatim, such as a trailing one, is still allowed, since every later
/// expansion leaves it unchanged.
pub fn ensure_descriptor_env_wire_safe(descriptor: &Descriptor) -> Result<()> {
    if let Some(env) = &descriptor.env {
        ensure_env_map_wire_safe(env, "the dataflow")?;
    }
    for node in &descriptor.nodes {
        ensure_node_env_wire_safe(node)?;
    }
    Ok(())
}

/// [`ensure_descriptor_env_wire_safe`] for a single node, as sent by
/// `dora node add` / `dora node replace`.
pub fn ensure_node_env_wire_safe(node: &Node) -> Result<()> {
    if let Some(env) = &node.env {
        ensure_env_map_wire_safe(env, format_args!("node `{}`", node.id))?;
    }
    Ok(())
}

fn ensure_env_map_wire_safe(
    env: &BTreeMap<String, EnvValue>,
    origin: impl std::fmt::Display,
) -> Result<()> {
    for (key, value) in env {
        if let EnvValue::String(value) = value
            && has_expandable_dollar(value)
        {
            bail!(
                "env `{key}` of {origin} contains a `$` after expansion, which the \
                 coordinator and daemon would expand again against THEIR OWN \
                 environment, so the node would not receive this value. (The value \
                 is not printed, as it may be a secret.)\n\n  \
                 hint: a variable referenced from the YAML must not itself contain \
                 `$VAR`, `${{VAR}}` or `$$`; if the `$` is literal (e.g. in a \
                 password), pass the value to the node another way, such as a file."
            );
        }
    }
    Ok(())
}

/// Whether `shellexpand::env` (behind `with_expand_envs`) may change
/// `value`: it rewrites a `$` followed by `{`, by a variable-name
/// character, or by a second `$` (an escape that collapses to one), and
/// keeps any other `$` as is. A `${` with no closing brace is also kept,
/// but is refused here too rather than mirroring that detail.
fn has_expandable_dollar(value: &str) -> bool {
    value.match_indices('$').any(|(i, _)| {
        value[i + 1..]
            .chars()
            .next()
            .is_some_and(|c| c == '{' || c == '$' || c == '_' || c.is_alphanumeric())
    })
}

/// What a coordinator or daemon released before the lossless `EnvValue`
/// fix turns the string `value` into, if that differs from `value`. Their
/// untagged enum tried `Bool`, `Integer` and `Float` (each parsing the
/// string) before `String`.
fn legacy_coercion(value: &str) -> Option<String> {
    let coerced = if let Ok(b) = value.parse::<bool>() {
        b.to_string()
    } else if let Ok(i) = value.parse::<i64>() {
        i.to_string()
    } else if let Ok(f) = value.parse::<f64>() {
        f.to_string()
    } else {
        return None;
    };
    (coerced != value).then_some(coerced)
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
        assert_child_test_passes(
            "self_referential_child",
            ("DORA_TEST_2919_SELFREF", "${DORA_TEST_2919_SELFREF}"),
            "the self-referential value was NOT rejected — a `$` reached the wire",
        );
    }

    /// Re-run this test binary for the single test `child` with `key=value` set,
    /// and assert that it passed.
    fn assert_child_test_passes(child: &str, (key, value): (&str, &str), failure: &str) {
        let exe = std::env::current_exe().expect("test binary path");
        let output = std::process::Command::new(exe)
            .args(["--exact", &format!("env_overrides::tests::{child}")])
            .env(key, value)
            .env("RUST_TEST_NOCAPTURE", "0")
            .output()
            .expect("failed to re-run test binary");
        assert!(
            output.status.success(),
            "{failure}.\nstdout:\n{}\nstderr:\n{}",
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

    /// Coordinators and daemons released before the lossless `EnvValue`
    /// fix coerce numeric-looking values in transit, and still accept a
    /// newer CLI. Better to refuse than to hand a node `1.1` when the
    /// operator typed `1.10`; `inf`/`NaN` would not even decode.
    #[test]
    fn rejects_values_the_wire_encoding_would_coerce() {
        for value in ["VERSION=1.10", "ZIP=01234", "SCI=1e5", "LIMIT=inf", "X=NaN"] {
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

    /// dora-rs/dora#3947: a YAML value is expanded CLI-side, and again by
    /// each process that decodes the descriptor; any `$` the first pass
    /// leaves that a later pass would rewrite must be refused.
    #[test]
    fn detects_dollars_a_later_expansion_would_rewrite() {
        for value in ["${HOME}", "$HOME", "p$ssw0rd", "a$_b", "$$", "x${unclosed"] {
            assert!(has_expandable_dollar(value), "`{value}` must be refused");
        }
        for value in ["", "plain", "5$", "$-", "a $ b", "$/path", "$}"] {
            assert!(!has_expandable_dollar(value), "`{value}` is stable");
        }
    }

    #[test]
    fn rejects_descriptor_env_that_would_be_expanded_again() {
        let dataflow_level: Descriptor =
            Descriptor::parse(b"env:\n  PW: \"p$$ssw0rd\"\nnodes: []\n".to_vec()).unwrap();
        assert_eq!(
            dataflow_level.env.as_ref().unwrap()["PW"],
            EnvValue::String("p$ssw0rd".into()),
            "test premise: `$$` is read as a literal `$`"
        );
        let err = ensure_descriptor_env_wire_safe(&dataflow_level).unwrap_err();
        assert!(err.to_string().contains("`PW`"), "{err}");

        let node_level: Descriptor = Descriptor::parse(
            b"nodes:\n  - id: probe\n    path: probe.bin\n    env:\n      PW: \"p$$ssw0rd\"\n"
                .to_vec(),
        )
        .unwrap();
        let err = ensure_descriptor_env_wire_safe(&node_level).unwrap_err();
        assert!(err.to_string().contains("node `probe`"), "{err}");
    }

    #[test]
    fn accepts_descriptor_env_that_survives_another_expansion() {
        let descriptor: Descriptor = Descriptor::parse(
            b"env:\n  PRICE: 5$\n  PORT: 8080\nnodes:\n  - id: probe\n    path: probe.bin\n    env:\n      OPTS: --level=debug\n"
                .to_vec(),
        )
        .unwrap();
        ensure_descriptor_env_wire_safe(&descriptor).expect("these values are wire-safe");
    }

    /// The reproduction from the issue: `X: ${FOO}` with
    /// `FOO='${DORA_AUTH_TOKEN}'` in the CLI's environment puts
    /// `${DORA_AUTH_TOKEN}` on the wire for the daemon to expand. Runs in
    /// a child process, as `set_var` is `unsafe` and would race siblings.
    #[test]
    fn yaml_reference_to_a_variable_holding_a_reference_is_rejected() {
        assert_child_test_passes(
            "yaml_reference_child",
            ("DORA_TEST_3947_INDIRECT", "${DORA_AUTH_TOKEN}"),
            "the indirect reference was NOT rejected",
        );
    }

    /// Child of `yaml_reference_to_a_variable_holding_a_reference_is_rejected`;
    /// a no-op when run directly.
    #[test]
    fn yaml_reference_child() {
        if std::env::var("DORA_TEST_3947_INDIRECT").is_err() {
            return; // not the child invocation
        }
        let descriptor: Descriptor =
            Descriptor::parse(b"env:\n  X: ${DORA_TEST_3947_INDIRECT}\nnodes: []\n".to_vec())
                .unwrap();
        assert_eq!(
            descriptor.env.as_ref().unwrap()["X"],
            EnvValue::String("${DORA_AUTH_TOKEN}".into()),
            "child env not set up"
        );
        assert!(ensure_descriptor_env_wire_safe(&descriptor).is_err());
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
