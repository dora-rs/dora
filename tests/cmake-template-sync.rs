//! The cmake config templates exist as three copies: the canonical
//! `apis/c/node/cmake/` plus crate-local copies in `apis/c++/node/cmake/`
//! and `apis/c++/operator/cmake/`. The copies exist because `cargo package`
//! cannot include files from outside a crate's directory, so each published
//! crate must carry its own. Nothing else keeps them in sync — this test
//! does.

use std::path::Path;

#[test]
fn cmake_templates_are_in_sync() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let canonical_dir = root.join("apis/c/node/cmake");
    let copy_dirs = [
        root.join("apis/c++/node/cmake"),
        root.join("apis/c++/operator/cmake"),
    ];

    for template in ["dora-api-config.cmake.in", "dora-api-version.cmake.in"] {
        let canonical = std::fs::read_to_string(canonical_dir.join(template))
            .unwrap_or_else(|e| panic!("failed to read canonical {template}: {e}"));
        for dir in &copy_dirs {
            let copy = std::fs::read_to_string(dir.join(template))
                .unwrap_or_else(|e| panic!("failed to read {}/{template}: {e}", dir.display()));
            assert_eq!(
                canonical,
                copy,
                "{}/{template} has drifted from apis/c/node/cmake/{template}; \
                 edit the canonical file and copy it to both c++ crates",
                dir.display()
            );
        }
    }
}
