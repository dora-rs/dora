//
// Copyright (c) 2025 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

use cfg_aliases::cfg_aliases;

fn main() {
    // these aliases should at least be included in the same aliases of Nix crate:
    // ___________________
    // |                 |
    // |  Nix aliases    |
    // |  ___________    |
    // |  |   Our   |    |
    // |  | aliases |    |
    // |  |_________|    |
    // |_________________|
    cfg_aliases! {
        dragonfly: { target_os = "dragonfly" },
        ios: { target_os = "ios" },
        freebsd: { target_os = "freebsd" },
        macos: { target_os = "macos" },
        netbsd: { target_os = "netbsd" },
        openbsd: { target_os = "openbsd" },
        watchos: { target_os = "watchos" },
        tvos: { target_os = "tvos" },
        visionos: { target_os = "visionos" },

        apple_targets: { any(ios, macos, watchos, tvos, visionos) },
        bsd: { any(freebsd, dragonfly, netbsd, openbsd, apple_targets) },

        // we use this alias to detect platforms that
        // don't support advisory file locking on tmpfs
        shm_external_lockfile: { any(bsd, target_os = "redox") },
    }

    println!("cargo:rustc-check-cfg=cfg(apple_targets)");
    println!("cargo:rustc-check-cfg=cfg(bsd)");
    println!("cargo:rustc-check-cfg=cfg(shm_external_lockfile)");
}
