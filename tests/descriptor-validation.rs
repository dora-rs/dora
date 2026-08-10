use dora_core::descriptor::{Descriptor, DescriptorExt};
use std::path::{Path, PathBuf};

fn case_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/descriptor-validation/cases")
        .join(name)
}

fn read_case(name: &str) -> Descriptor {
    Descriptor::blocking_read(&case_path(name)).expect("test descriptor should parse")
}

fn case_error(name: &str) -> String {
    let descriptor = read_case(name);
    let result = if name.starts_with("invalid-module-") {
        descriptor
            .expand(case_path(name).parent().unwrap())
            .map(|_| ())
    } else {
        descriptor.resolve_aliases_and_set_defaults().map(|_| ())
    };
    format!(
        "{:#}",
        result.expect_err("descriptor case should fail validation")
    )
}

#[test]
fn valid_descriptor_field_cases_resolve() {
    for name in [
        "valid-standard.yml",
        "valid-custom.yml",
        "valid-runtime.yml",
        "valid-operator.yml",
        "valid-ros2-metadata.yml",
    ] {
        read_case(name)
            .resolve_aliases_and_set_defaults()
            .unwrap_or_else(|err| panic!("{name} should resolve: {err:#}"));
    }

    read_case("valid-module.yml")
        .expand(case_path("valid-module.yml").parent().unwrap())
        .expect("valid module descriptor should expand");
}

#[test]
fn invalid_descriptor_field_cases_report_rejected_fields() {
    for (name, expected) in [
        ("invalid-standard-params.yml", "params"),
        ("invalid-custom-outputs.yml", "outputs"),
        ("invalid-runtime-outputs.yml", "outputs"),
        ("invalid-operator-outputs.yml", "outputs"),
        ("invalid-ros2-git.yml", "git"),
        ("invalid-module-build.yml", "build"),
    ] {
        let error = case_error(name);
        assert!(
            error.contains(expected) && error.contains("not allowed"),
            "{name} error did not mention rejected field `{expected}`:\n{error}"
        );
    }
}
