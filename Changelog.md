# Changelog

# v0.3.14 (2026-01-08)

## What's Changed

- docs: add kornia nodes and example in Readme by @edgarriba in https://github.com/adora-rs/adora/pull/1046
- Fix CI/CD by removing bitsandbytes from adora-phi4 as well as increase disk space for adora-phi4 by @haixuanTao in https://github.com/adora-rs/adora/pull/1051
- Add vggt based URDF visualisation by @haixuanTao in https://github.com/adora-rs/adora/pull/1044
- docs: fix speech-to-speech example README by @chrislearn in https://github.com/adora-rs/adora/pull/1052
- Add exif to avif for encoding focal information with avif file by @haixuanTao in https://github.com/adora-rs/adora/pull/1045
- Made some changes to make clippy happy by @chrislearn in https://github.com/adora-rs/adora/pull/1053
- CLI Rework by @sjfhsjfh in https://github.com/adora-rs/adora/pull/979
- Removing support for macos x86 in order to unpin macos version by @haixuanTao in https://github.com/adora-rs/adora/pull/1048
- fix: in c++ ros2 example, too few ticks. by @drindr in https://github.com/adora-rs/adora/pull/1060
- Fix buffer not printing because it contains legacy ERROR message by @haixuanTao in https://github.com/adora-rs/adora/pull/1062
- Add back pyo3 special method by @haixuanTao in https://github.com/adora-rs/adora/pull/1061
- convert chinese character to pinyin in rerun by @haixuanTao in https://github.com/adora-rs/adora/pull/1049
- Fix weird daemon folder by @haixuanTao in https://github.com/adora-rs/adora/pull/1064
- Revert "Fix weird daemon folder" by @phil-opp in https://github.com/adora-rs/adora/pull/1065
- Add dataset collection node for adora by @ShashwatPatil in https://github.com/adora-rs/adora/pull/1041
- Remove rust--toolchain.toml and use rust-version in Cargo.toml by @chrislearn in https://github.com/adora-rs/adora/pull/1071
- Fix many `cargo clippy` warning by @chrislearn in https://github.com/adora-rs/adora/pull/1070
- Add proper documentation to `adora-node-api` crate by @phil-opp in https://github.com/adora-rs/adora/pull/1056
- [node-hub] opencv-plot: allow specifying encoding and bbox format in uppercase letters by @dieu-detruit in https://github.com/adora-rs/adora/pull/1075
- python api: added py.typed by @dieu-detruit in https://github.com/adora-rs/adora/pull/1078
- Fix all clippy warning by @chrislearn in https://github.com/adora-rs/adora/pull/1076
- Fix: Create working directory if it doesn't exist by @phil-opp in https://github.com/adora-rs/adora/pull/1066
- doc: Node & Descriptor by @sjfhsjfh in https://github.com/adora-rs/adora/pull/1069
- Improve build logging by @phil-opp in https://github.com/adora-rs/adora/pull/1067
- chore: update json schema by @sjfhsjfh in https://github.com/adora-rs/adora/pull/1080
- Document public `DYNAMIC_SOURCE` constant by @phil-opp in https://github.com/adora-rs/adora/pull/1082
- Automate schema regeneration through CI by @phil-opp in https://github.com/adora-rs/adora/pull/1083
- Update `schemars` crate to `v1.0.4` by @phil-opp in https://github.com/adora-rs/adora/pull/1085
- Update JSON schema for `adora-core` by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1086
- Minor tweaks for JSON schema regeneration PR by @phil-opp in https://github.com/adora-rs/adora/pull/1087
- Update JSON schema for `adora-core` by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1088
- Error if node with `git` field was not built before dataflow start by @phil-opp in https://github.com/adora-rs/adora/pull/1089
- Update rust edition to 2024 by @chrislearn in https://github.com/adora-rs/adora/pull/1072
- Add a MSRV CI job by @phil-opp in https://github.com/adora-rs/adora/pull/1094
- Updates to rerun 0.24.0 + message type proposal by @rozgo in https://github.com/adora-rs/adora/pull/1096
- Add mcp-host node and mcp-server node by @chrislearn in https://github.com/adora-rs/adora/pull/1063
- Update speech-to-text doc by @chrislearn in https://github.com/adora-rs/adora/pull/1097
- Add default readme for crates by @chrislearn in https://github.com/adora-rs/adora/pull/1102
- Improve `renovate` config to open separate PRs for Rust and Python by @phil-opp in https://github.com/adora-rs/adora/pull/1107
- Run cargo update by @phil-opp in https://github.com/adora-rs/adora/pull/1109
- Run CI also when PRs are marked as ready for review by @phil-opp in https://github.com/adora-rs/adora/pull/1130
- Feat/build arg with space by @drindr in https://github.com/adora-rs/adora/pull/1103
- feat/path-with-env by @drindr in https://github.com/adora-rs/adora/pull/1104
- Update Rust crate tracing-subscriber to v0.3.20 [SECURITY] by @renovate[bot] in https://github.com/adora-rs/adora/pull/1121
- Downgrade `opentelemetry` to `v0.29` again in `adora-metrics` by @phil-opp in https://github.com/adora-rs/adora/pull/1132
- Use debug logging in CI to debug `uv` lock deadlock by @phil-opp in https://github.com/adora-rs/adora/pull/1137
- Fix typo in variable name by @phil-opp in https://github.com/adora-rs/adora/pull/1136
- Fix build command execution by @Mivik in https://github.com/adora-rs/adora/pull/1140
- add model endpoint to openai proxy server by @haixuanTao in https://github.com/adora-rs/adora/pull/1144
- Auto-cancel previous CI jobs on new pushes by @phil-opp in https://github.com/adora-rs/adora/pull/1135
- Move node-hub to separate repo by @phil-opp in https://github.com/adora-rs/adora/pull/1139
- chore: Update Cargo.lock by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1138
- Replace `names` dependency with `petname` crate by @phil-opp in https://github.com/adora-rs/adora/pull/1146
- Remove `communication-layer-pub-sub` crate by @phil-opp in https://github.com/adora-rs/adora/pull/1145
- Fix Cargo.lock by @phil-opp in https://github.com/adora-rs/adora/pull/1147
- Feat: adoraflow api by @heyong4725 in https://github.com/adora-rs/adora/pull/1106
- expose the node_config method in python's api by @drindr in https://github.com/adora-rs/adora/pull/1152
- Fix typos by @haixuanTao in https://github.com/adora-rs/adora/pull/1153
- chore: Update Cargo.lock by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1150
- Fix timer issue for multi dynamic nodes by @haixuanTao in https://github.com/adora-rs/adora/pull/1151
- Bump adora to v0.3.13 by @haixuanTao in https://github.com/adora-rs/adora/pull/1154

## New Contributors

- @edgarriba made their first contribution in https://github.com/adora-rs/adora/pull/1046
- @chrislearn made their first contribution in https://github.com/adora-rs/adora/pull/1052
- @drindr made their first contribution in https://github.com/adora-rs/adora/pull/1060
- @dieu-detruit made their first contribution in https://github.com/adora-rs/adora/pull/1075
- @github-actions[bot] made their first contribution in https://github.com/adora-rs/adora/pull/1086
- @rozgo made their first contribution in https://github.com/adora-rs/adora/pull/1096

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.12...v0.3.13

# v0.3.13 (2025-06-30)

## What's Changed

- docs: add kornia nodes and example in Readme by @edgarriba in https://github.com/adora-rs/adora/pull/1046
- Fix CI/CD by removing bitsandbytes from adora-phi4 as well as increase disk space for adora-phi4 by @haixuanTao in https://github.com/adora-rs/adora/pull/1051
- Add vggt based URDF visualisation by @haixuanTao in https://github.com/adora-rs/adora/pull/1044
- docs: fix speech-to-speech example README by @chrislearn in https://github.com/adora-rs/adora/pull/1052
- Add exif to avif for encoding focal information with avif file by @haixuanTao in https://github.com/adora-rs/adora/pull/1045
- Made some changes to make clippy happy by @chrislearn in https://github.com/adora-rs/adora/pull/1053
- CLI Rework by @sjfhsjfh in https://github.com/adora-rs/adora/pull/979
- Removing support for macos x86 in order to unpin macos version by @haixuanTao in https://github.com/adora-rs/adora/pull/1048
- fix: in c++ ros2 example, too few ticks. by @drindr in https://github.com/adora-rs/adora/pull/1060
- Fix buffer not printing because it contains legacy ERROR message by @haixuanTao in https://github.com/adora-rs/adora/pull/1062
- Add back pyo3 special method by @haixuanTao in https://github.com/adora-rs/adora/pull/1061
- convert chinese character to pinyin in rerun by @haixuanTao in https://github.com/adora-rs/adora/pull/1049
- Fix weird daemon folder by @haixuanTao in https://github.com/adora-rs/adora/pull/1064
- Revert "Fix weird daemon folder" by @phil-opp in https://github.com/adora-rs/adora/pull/1065
- Add dataset collection node for adora by @ShashwatPatil in https://github.com/adora-rs/adora/pull/1041
- Remove rust--toolchain.toml and use rust-version in Cargo.toml by @chrislearn in https://github.com/adora-rs/adora/pull/1071
- Fix many `cargo clippy` warning by @chrislearn in https://github.com/adora-rs/adora/pull/1070
- Add proper documentation to `adora-node-api` crate by @phil-opp in https://github.com/adora-rs/adora/pull/1056
- [node-hub] opencv-plot: allow specifying encoding and bbox format in uppercase letters by @dieu-detruit in https://github.com/adora-rs/adora/pull/1075
- python api: added py.typed by @dieu-detruit in https://github.com/adora-rs/adora/pull/1078
- Fix all clippy warning by @chrislearn in https://github.com/adora-rs/adora/pull/1076
- Fix: Create working directory if it doesn't exist by @phil-opp in https://github.com/adora-rs/adora/pull/1066
- doc: Node & Descriptor by @sjfhsjfh in https://github.com/adora-rs/adora/pull/1069
- Improve build logging by @phil-opp in https://github.com/adora-rs/adora/pull/1067
- chore: update json schema by @sjfhsjfh in https://github.com/adora-rs/adora/pull/1080
- Document public `DYNAMIC_SOURCE` constant by @phil-opp in https://github.com/adora-rs/adora/pull/1082
- Automate schema regeneration through CI by @phil-opp in https://github.com/adora-rs/adora/pull/1083
- Update `schemars` crate to `v1.0.4` by @phil-opp in https://github.com/adora-rs/adora/pull/1085
- Update JSON schema for `adora-core` by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1086
- Minor tweaks for JSON schema regeneration PR by @phil-opp in https://github.com/adora-rs/adora/pull/1087
- Update JSON schema for `adora-core` by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1088
- Error if node with `git` field was not built before dataflow start by @phil-opp in https://github.com/adora-rs/adora/pull/1089
- Update rust edition to 2024 by @chrislearn in https://github.com/adora-rs/adora/pull/1072
- Add a MSRV CI job by @phil-opp in https://github.com/adora-rs/adora/pull/1094
- Updates to rerun 0.24.0 + message type proposal by @rozgo in https://github.com/adora-rs/adora/pull/1096
- Add mcp-host node and mcp-server node by @chrislearn in https://github.com/adora-rs/adora/pull/1063
- Update speech-to-text doc by @chrislearn in https://github.com/adora-rs/adora/pull/1097
- Add default readme for crates by @chrislearn in https://github.com/adora-rs/adora/pull/1102
- Improve `renovate` config to open separate PRs for Rust and Python by @phil-opp in https://github.com/adora-rs/adora/pull/1107
- Run cargo update by @phil-opp in https://github.com/adora-rs/adora/pull/1109
- Run CI also when PRs are marked as ready for review by @phil-opp in https://github.com/adora-rs/adora/pull/1130
- Feat/build arg with space by @drindr in https://github.com/adora-rs/adora/pull/1103
- feat/path-with-env by @drindr in https://github.com/adora-rs/adora/pull/1104
- Update Rust crate tracing-subscriber to v0.3.20 [SECURITY] by @renovate[bot] in https://github.com/adora-rs/adora/pull/1121
- Downgrade `opentelemetry` to `v0.29` again in `adora-metrics` by @phil-opp in https://github.com/adora-rs/adora/pull/1132
- Use debug logging in CI to debug `uv` lock deadlock by @phil-opp in https://github.com/adora-rs/adora/pull/1137
- Fix typo in variable name by @phil-opp in https://github.com/adora-rs/adora/pull/1136
- Fix build command execution by @Mivik in https://github.com/adora-rs/adora/pull/1140
- add model endpoint to openai proxy server by @haixuanTao in https://github.com/adora-rs/adora/pull/1144
- Auto-cancel previous CI jobs on new pushes by @phil-opp in https://github.com/adora-rs/adora/pull/1135
- Move node-hub to separate repo by @phil-opp in https://github.com/adora-rs/adora/pull/1139
- chore: Update Cargo.lock by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1138
- Replace `names` dependency with `petname` crate by @phil-opp in https://github.com/adora-rs/adora/pull/1146
- Remove `communication-layer-pub-sub` crate by @phil-opp in https://github.com/adora-rs/adora/pull/1145
- Fix Cargo.lock by @phil-opp in https://github.com/adora-rs/adora/pull/1147
- Feat: adoraflow api by @heyong4725 in https://github.com/adora-rs/adora/pull/1106
- expose the node_config method in python's api by @drindr in https://github.com/adora-rs/adora/pull/1152
- Fix typos by @haixuanTao in https://github.com/adora-rs/adora/pull/1153
- chore: Update Cargo.lock by @github-actions[bot] in https://github.com/adora-rs/adora/pull/1150
- Fix timer issue for multi dynamic nodes by @haixuanTao in https://github.com/adora-rs/adora/pull/1151

## New Contributors

- @edgarriba made their first contribution in https://github.com/adora-rs/adora/pull/1046
- @chrislearn made their first contribution in https://github.com/adora-rs/adora/pull/1052
- @drindr made their first contribution in https://github.com/adora-rs/adora/pull/1060
- @dieu-detruit made their first contribution in https://github.com/adora-rs/adora/pull/1075
- @github-actions[bot] made their first contribution in https://github.com/adora-rs/adora/pull/1086
- @rozgo made their first contribution in https://github.com/adora-rs/adora/pull/1096

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.12...v0.3.13

## v0.3.12 (2025-06-30)

## What's Changed

- Implemented adora-cotracker node by @ShashwatPatil in https://github.com/adora-rs/adora/pull/931
- Minor fix and add boxes2d example to facebook/cotracker by @haixuanTao in https://github.com/adora-rs/adora/pull/950
- Update Rust crate tokio to v1.44.2 [SECURITY] by @renovate in https://github.com/adora-rs/adora/pull/951
- Post 3.11 release fix by @haixuanTao in https://github.com/adora-rs/adora/pull/954
- Bump crossbeam-channel from 0.5.14 to 0.5.15 by @dependabot in https://github.com/adora-rs/adora/pull/959
- Added E ruff flag for pydocstyle by @7SOMAY in https://github.com/adora-rs/adora/pull/958
- Revert "Added E ruff flag for better code quality [skip ci]" by @haixuanTao in https://github.com/adora-rs/adora/pull/968
- Ease of use changes in benches for issue #957 by @Ignavar in https://github.com/adora-rs/adora/pull/969
- Reachy cotracker by @haixuanTao in https://github.com/adora-rs/adora/pull/972
- Improve rav1e by @haixuanTao in https://github.com/adora-rs/adora/pull/974
- Fix pyrealsense by @haixuanTao in https://github.com/adora-rs/adora/pull/973
- Added Self Uninstall Command by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/944
- Improve benchmark implementation & Add warning for discarding events by @Mivik in https://github.com/adora-rs/adora/pull/971
- docs: Updated README: Added comprehensive usage documentation with vi… by @LeonRust in https://github.com/adora-rs/adora/pull/983
- Fix rerun-viewer example. by @francocipollone in https://github.com/adora-rs/adora/pull/989
- docs: add license badge by @Radovenchyk in https://github.com/adora-rs/adora/pull/996
- Disable sccache for `musllinux` jobs by @haixuanTao in https://github.com/adora-rs/adora/pull/1000
- Remove unused sysinfo monitor by @Mivik in https://github.com/adora-rs/adora/pull/1007
- Refactor Python CUDA IPC API by @Mivik in https://github.com/adora-rs/adora/pull/1002
- fix terminal not printing stdout on nvml warning by @haixuanTao in https://github.com/adora-rs/adora/pull/1008
- Fix issue #1006: [Brief description of the fix] by @sohamukute in https://github.com/adora-rs/adora/pull/1013
- Improving so100 usability by @haixuanTao in https://github.com/adora-rs/adora/pull/988
- Add adora-mediapipe node for quick human pose estimation by @haixuanTao in https://github.com/adora-rs/adora/pull/986
- Bump torch to 2.7 by @haixuanTao in https://github.com/adora-rs/adora/pull/1015
- refactor(tracing): use builder style by @sjfhsjfh in https://github.com/adora-rs/adora/pull/1009
- Fix spawning runtime through python when it is installed with pip by @haixuanTao in https://github.com/adora-rs/adora/pull/1011
- chore(deps): update dependency numpy to v2 by @renovate in https://github.com/adora-rs/adora/pull/1014
- Fix error when multiple visualization key is active and when urdf_transform env variable is not present by @haixuanTao in https://github.com/adora-rs/adora/pull/1016
- Update pyrealsense2 Dependencies for L515 Support and Fix README wget Link by @kingchou007 in https://github.com/adora-rs/adora/pull/1021
- Minor fix for mujoco sim by @haixuanTao in https://github.com/adora-rs/adora/pull/1023
- adora-mujoco simulation node with example for controlling any arm by @ShashwatPatil in https://github.com/adora-rs/adora/pull/1012
- fix ros CI/CD by @haixuanTao in https://github.com/adora-rs/adora/pull/1027
- adora-vggt by @haixuanTao in https://github.com/adora-rs/adora/pull/1024
- Adding vision to openai server by @haixuanTao in https://github.com/adora-rs/adora/pull/1025
- Revert "Adding vision to openai server" by @haixuanTao in https://github.com/adora-rs/adora/pull/1031
- Expose AllInputClosed message as a Stop message by @haixuanTao in https://github.com/adora-rs/adora/pull/1026
- Add support for git repository sources for nodes by @phil-opp in https://github.com/adora-rs/adora/pull/901
- Adding vision to rust openai proxy server by @haixuanTao in https://github.com/adora-rs/adora/pull/1033
- Add automatic robot descriptions URDF retrieval from https://github.com/robot-descriptions/robot_descriptions.py by @haixuanTao in https://github.com/adora-rs/adora/pull/1032

## New Contributors

- @Mivik made their first contribution in https://github.com/adora-rs/adora/pull/971
- @francocipollone made their first contribution in https://github.com/adora-rs/adora/pull/989
- @sohamukute made their first contribution in https://github.com/adora-rs/adora/pull/1013
- @sjfhsjfh made their first contribution in https://github.com/adora-rs/adora/pull/1009
- @kingchou007 made their first contribution in https://github.com/adora-rs/adora/pull/1021

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.11...v0.3.12

## v0.3.11 (2025-04-07)

## What's Changed

- Post adora 0.3.10 release fix by @haixuanTao in https://github.com/adora-rs/adora/pull/804
- Add windows release for rust nodes by @haixuanTao in https://github.com/adora-rs/adora/pull/805
- Add Node Table into README.md by @haixuanTao in https://github.com/adora-rs/adora/pull/808
- update adora yaml json schema validator by @haixuanTao in https://github.com/adora-rs/adora/pull/809
- Improve readme support matrix readability by @haixuanTao in https://github.com/adora-rs/adora/pull/810
- Clippy automatic fixes applied by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/812
- Improve documentation on adding new node to the node-hub by @haixuanTao in https://github.com/adora-rs/adora/pull/820
- #807 Fixed by @7SOMAY in https://github.com/adora-rs/adora/pull/818
- Applied Ruff pydocstyle to adora by @Mati-ur-rehman-017 in https://github.com/adora-rs/adora/pull/831
- Related to adora-bot issue assignment by @MunishMummadi in https://github.com/adora-rs/adora/pull/840
- Add adora-lerobot node into adora by @Ignavar in https://github.com/adora-rs/adora/pull/834
- CI: Permit issue modifications for issue assign job by @phil-opp in https://github.com/adora-rs/adora/pull/848
- Fix: Set variables outside bash script to prevent injection by @phil-opp in https://github.com/adora-rs/adora/pull/849
- Replacing Deprecated functions of pyo3 by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/838
- Add noise filtering on whisper to be able to use speakers by @haixuanTao in https://github.com/adora-rs/adora/pull/847
- Add minimal Dockerfile with Python and uv for easy onboarding by @Krishnadubey1008 in https://github.com/adora-rs/adora/pull/843
- More compact readme with example section by @haixuanTao in https://github.com/adora-rs/adora/pull/855
- Create docker-image.yml by @haixuanTao in https://github.com/adora-rs/adora/pull/857
- Multi platform docker by @haixuanTao in https://github.com/adora-rs/adora/pull/858
- change: `adora/node-hub/README.md` by @MunishMummadi in https://github.com/adora-rs/adora/pull/862
- Added adora-phi4 inside node-hub by @7SOMAY in https://github.com/adora-rs/adora/pull/861
- node-hub: Added adora-magma node by @MunishMummadi in https://github.com/adora-rs/adora/pull/853
- Added the adora-llama-cpp-python node by @ShashwatPatil in https://github.com/adora-rs/adora/pull/850
- Adding in some missing types and test cases within arrow convert crate by @Ignavar in https://github.com/adora-rs/adora/pull/864
- Migrate robots from adora-lerobot to adora repository by @rahat2134 in https://github.com/adora-rs/adora/pull/868
- Applied pyupgrade style by @Mati-ur-rehman-017 in https://github.com/adora-rs/adora/pull/876
- Adding additional llm in tests by @haixuanTao in https://github.com/adora-rs/adora/pull/873
- Adora transformer node by @ShashwatPatil in https://github.com/adora-rs/adora/pull/870
- Using macros in Arrow Conversion by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/877
- Adding run command within python API by @haixuanTao in https://github.com/adora-rs/adora/pull/875
- Added f16 type conversion by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/886
- Added "PERF" flag inside node-hub by @7SOMAY in https://github.com/adora-rs/adora/pull/880
- Added quality ruff-flags for better code quality by @7SOMAY in https://github.com/adora-rs/adora/pull/888
- Add llm benchmark by @haixuanTao in https://github.com/adora-rs/adora/pull/881
- Implement `into_vec_f64(&ArrowData) -> Vec<f64)` conversion function by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/893
- Adding virtual env within adora build command by @haixuanTao in https://github.com/adora-rs/adora/pull/895
- Adding metrics for node api by @haixuanTao in https://github.com/adora-rs/adora/pull/903
- Made UI interface for input in adora, using Gradio by @ShashwatPatil in https://github.com/adora-rs/adora/pull/891
- Add chinese voice support by @haixuanTao in https://github.com/adora-rs/adora/pull/902
- Made conversion generic by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/908
- Added husky simulation in Mujoco and gamepad node by @ShashwatPatil in https://github.com/adora-rs/adora/pull/906
- use `cargo-dist` tool for adora-cli releases by @Hennzau in https://github.com/adora-rs/adora/pull/916
- Implementing Self update by @Shar-jeel-Sajid in https://github.com/adora-rs/adora/pull/920
- Fix: RUST_LOG=. adora run bug by @starlitxiling in https://github.com/adora-rs/adora/pull/924
- Added adora-mistral-rs node in node-hub for inference in rust by @Ignavar in https://github.com/adora-rs/adora/pull/910
- Fix reachy left arm by @haixuanTao in https://github.com/adora-rs/adora/pull/907
- Functions for sending and receiving data using Arrow::FFI by @Mati-ur-rehman-017 in https://github.com/adora-rs/adora/pull/918
- Adding `recv_async` adora method to retrieve data in python async by @haixuanTao in https://github.com/adora-rs/adora/pull/909
- Update: README.md of the node hub by @Choudhry18 in https://github.com/adora-rs/adora/pull/929
- Fix magma by @haixuanTao in https://github.com/adora-rs/adora/pull/926
- Add support for mask in rerun by @haixuanTao in https://github.com/adora-rs/adora/pull/927
- Bump array-init-cursor from 0.2.0 to 0.2.1 by @dependabot in https://github.com/adora-rs/adora/pull/933
- Enhance Zenoh Integration Documentation by @NageshMandal in https://github.com/adora-rs/adora/pull/935
- Support av1 by @haixuanTao in https://github.com/adora-rs/adora/pull/932
- Bump adora v0.3.11 by @haixuanTao in https://github.com/adora-rs/adora/pull/948

## New Contributors

- @Shar-jeel-Sajid made their first contribution in https://github.com/adora-rs/adora/pull/812
- @7SOMAY made their first contribution in https://github.com/adora-rs/adora/pull/818
- @Mati-ur-rehman-017 made their first contribution in https://github.com/adora-rs/adora/pull/831
- @MunishMummadi made their first contribution in https://github.com/adora-rs/adora/pull/840
- @Ignavar made their first contribution in https://github.com/adora-rs/adora/pull/834
- @Krishnadubey1008 made their first contribution in https://github.com/adora-rs/adora/pull/843
- @ShashwatPatil made their first contribution in https://github.com/adora-rs/adora/pull/850
- @rahat2134 made their first contribution in https://github.com/adora-rs/adora/pull/868
- @Choudhry18 made their first contribution in https://github.com/adora-rs/adora/pull/929
- @NageshMandal made their first contribution in https://github.com/adora-rs/adora/pull/935

## v0.3.10 (2025-03-04)

## What's Changed

- Enables array based bounding boxes by @haixuanTao in https://github.com/adora-rs/adora/pull/772
- Fix typo in node version by @haixuanTao in https://github.com/adora-rs/adora/pull/773
- CI: Use `paths-ignore` instead of negated `paths` by @phil-opp in https://github.com/adora-rs/adora/pull/781
- Adding rerun connect options by @haixuanTao in https://github.com/adora-rs/adora/pull/782
- Forbid `/` in node IDs by @phil-opp in https://github.com/adora-rs/adora/pull/785
- Adding reachy and adora reachy demo by @haixuanTao in https://github.com/adora-rs/adora/pull/784
- Fix typo in reachy node by @haixuanTao in https://github.com/adora-rs/adora/pull/789
- Update dependency transformers to >=4.48.0,<=4.48.0 [SECURITY] - abandoned by @renovate in https://github.com/adora-rs/adora/pull/778
- Fix bounding box for rerun viewer and clear the viewer if no bounding box is detected by @haixuanTao in https://github.com/adora-rs/adora/pull/787
- Adding float for env variable and metadata parameters by @haixuanTao in https://github.com/adora-rs/adora/pull/786
- Limit pip release ci to strict minimum by @haixuanTao in https://github.com/adora-rs/adora/pull/791
- Add uv flag within start cli command by @haixuanTao in https://github.com/adora-rs/adora/pull/788
- Adding a test for checking on the latency when used timeout and queue at the same time by @haixuanTao in https://github.com/adora-rs/adora/pull/783
- Use zenoh for inter-daemon communication by @phil-opp in https://github.com/adora-rs/adora/pull/779
- Pin chrono version by @haixuanTao in https://github.com/adora-rs/adora/pull/797
- Add kokoro tts by @haixuanTao in https://github.com/adora-rs/adora/pull/794
- Pick place demo by @haixuanTao in https://github.com/adora-rs/adora/pull/793
- Bump pyo3 to 0.23 by @haixuanTao in https://github.com/adora-rs/adora/pull/798
- Faster node hub CI/CD by removing `free disk space on ubuntu` by @haixuanTao in https://github.com/adora-rs/adora/pull/801

## v0.3.9 (2025-02-06)

## What's Changed

- Making cli install the default api avoiding confusion on install by @haixuanTao in https://github.com/adora-rs/adora/pull/739
- Add description within visualisation by @haixuanTao in https://github.com/adora-rs/adora/pull/742
- Added depth image and data output for the adora-pyorbbecksdk node by @Ryu-Yang in https://github.com/adora-rs/adora/pull/740
- Improve speech to text example within the macOS ecosystem by @haixuanTao in https://github.com/adora-rs/adora/pull/741
- Rewrite python template to make them pip installable by @haixuanTao in https://github.com/adora-rs/adora/pull/744
- bump rerun version by @haixuanTao in https://github.com/adora-rs/adora/pull/743
- Replace pylint with ruff by @haixuanTao in https://github.com/adora-rs/adora/pull/756
- Make unknown output acceptable by @haixuanTao in https://github.com/adora-rs/adora/pull/755
- Improve Speech-to-Speech pipeline by better support for macOS and additional OutteTTS model by @haixuanTao in https://github.com/adora-rs/adora/pull/752
- Daemon: React to ctrl-c during connection setup by @phil-opp in https://github.com/adora-rs/adora/pull/758
- Use UV for the CI/CD by @haixuanTao in https://github.com/adora-rs/adora/pull/757
- chore: fix some typos in comment by @sunxunle in https://github.com/adora-rs/adora/pull/759
- Add ios lidar by @haixuanTao in https://github.com/adora-rs/adora/pull/762
- Print python stdout without buffer even for script by @haixuanTao in https://github.com/adora-rs/adora/pull/761
- chore: use workspace edition by @yjhmelody in https://github.com/adora-rs/adora/pull/764
- Add a uv flag to make it possible to automatically replace `pip` with `uv pip` and prepend run command with `uv run` by @haixuanTao in https://github.com/adora-rs/adora/pull/765
- Use mlx whisper instead of lightning-whisper by @haixuanTao in https://github.com/adora-rs/adora/pull/766
- Reduce silence duration in VAD by @haixuanTao in https://github.com/adora-rs/adora/pull/768
- Bump upload artifact version by @haixuanTao in https://github.com/adora-rs/adora/pull/769
- Add qwenvl2 5 by @haixuanTao in https://github.com/adora-rs/adora/pull/767

## New Contributors

- @Ryu-Yang made their first contribution in https://github.com/adora-rs/adora/pull/740
- @sunxunle made their first contribution in https://github.com/adora-rs/adora/pull/759
- @yjhmelody made their first contribution in https://github.com/adora-rs/adora/pull/764

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.8...v0.3.9

## Breaking Change

Inputs are now schedule fairly meaning that they will be now be received equally and not necessarily in chronological order. This enables to always be able to refresh input with the least latency between input.

## v0.3.8 (2024-12-06)

- Make node hub CI/CD cross platform by @haixuanTao in https://github.com/adora-rs/adora/pull/714
- Make node hub CI/CD cross architecture by @haixuanTao in https://github.com/adora-rs/adora/pull/716
- Make list an available type for metadata by @haixuanTao in https://github.com/adora-rs/adora/pull/721
- Add stdout logging by @haixuanTao in https://github.com/adora-rs/adora/pull/720
- Add an error when a node fails when using adora run by @haixuanTao in https://github.com/adora-rs/adora/pull/719
- Add pyarrow cuda zero copy helper by @haixuanTao in https://github.com/adora-rs/adora/pull/722
- feat: Add Adora-kit car Control in node-hub by @LyonRust in https://github.com/adora-rs/adora/pull/715
- Add yuv420 encoding to opencv-video-capture by @haixuanTao in https://github.com/adora-rs/adora/pull/725
- Change macOS CI runner to `macos-13` by @phil-opp in https://github.com/adora-rs/adora/pull/729
- Add eyre to pyo3 node by @haixuanTao in https://github.com/adora-rs/adora/pull/730
- Moving queue size and making node flume queue bigger by @haixuanTao in https://github.com/adora-rs/adora/pull/724
- Make python default for macos by @haixuanTao in https://github.com/adora-rs/adora/pull/731
- Modify the node queue Scheduler to make it able to schedule input fairly by @haixuanTao in https://github.com/adora-rs/adora/pull/728

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.7...v0.3.8

## v0.3.7 (2024-11-15)

## What's Changed

- Post release `0.3.6` small fix by @haixuanTao in https://github.com/adora-rs/adora/pull/638
- Changes to template by @XxChang in https://github.com/adora-rs/adora/pull/639
- Add appending to PATH instruction inside installation script by @Hennzau in https://github.com/adora-rs/adora/pull/641
- Make the benchmark run in release and at full speed by @Hennzau in https://github.com/adora-rs/adora/pull/644
- Use the new node syntax for examples dataflow by @Hennzau in https://github.com/adora-rs/adora/pull/643
- Improve beginner experience by @Hennzau in https://github.com/adora-rs/adora/pull/645
- improve node-hub pytest by @haixuanTao in https://github.com/adora-rs/adora/pull/640
- Fix not-null terminated string print within C template by @haixuanTao in https://github.com/adora-rs/adora/pull/654
- Raise error if adora-coordinator is not connected when calling `adora destroy` by @haixuanTao in https://github.com/adora-rs/adora/pull/655
- Coordinator stopped on bad control command by @Hennzau in https://github.com/adora-rs/adora/pull/650
- Add support for Qwenvl2 by @haixuanTao in https://github.com/adora-rs/adora/pull/646
- Fix distributed node by @haixuanTao in https://github.com/adora-rs/adora/pull/658
- Small install script update for bash by @haixuanTao in https://github.com/adora-rs/adora/pull/657
- Add additional image encoding by @haixuanTao in https://github.com/adora-rs/adora/pull/661
- `adora-echo` replicate the topic received with the topic send by @haixuanTao in https://github.com/adora-rs/adora/pull/663
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/656
- Bump pyo3 and arrow versions by @haixuanTao in https://github.com/adora-rs/adora/pull/667
- Fix ros2 bridge incompatibility with CI Ubuntu 24 and with pyo3 22 by @haixuanTao in https://github.com/adora-rs/adora/pull/670
- Add transformers version pinning for qwenvl2 by @haixuanTao in https://github.com/adora-rs/adora/pull/665
- Remove cli dataflow path check by @haixuanTao in https://github.com/adora-rs/adora/pull/662
- Better error handling for unknown output by @haixuanTao in https://github.com/adora-rs/adora/pull/675
- Fix llama recorder multi image recorder by @haixuanTao in https://github.com/adora-rs/adora/pull/677
- Adora openai server example by @haixuanTao in https://github.com/adora-rs/adora/pull/676
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/674
- Create Rust-based openai api proxy server in node hub by @phil-opp in https://github.com/adora-rs/adora/pull/678
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/679
- Update Rust crate hyper to v0.14.30 by @renovate in https://github.com/adora-rs/adora/pull/680
- Fix hanged coordinator when failing to connect to the daemon on destroy command by @haixuanTao in https://github.com/adora-rs/adora/pull/664
- Small example improvement using pyarrow assertion by @haixuanTao in https://github.com/adora-rs/adora/pull/669
- Fix adora list listing twice a stopping dataflow when using multiple daemon. by @haixuanTao in https://github.com/adora-rs/adora/pull/668
- Add package flake by @Ben-PH in https://github.com/adora-rs/adora/pull/685
- Add jpeg format to qwenvl2 by @haixuanTao in https://github.com/adora-rs/adora/pull/684
- Enable downloading remote dataflow by @haixuanTao in https://github.com/adora-rs/adora/pull/682
- Enable multiline build for better packaging of adora node. by @haixuanTao in https://github.com/adora-rs/adora/pull/683
- Bump rerun version to 0.18 by @haixuanTao in https://github.com/adora-rs/adora/pull/686
- Temporary fix qwenvl2 queue error by @haixuanTao in https://github.com/adora-rs/adora/pull/688
- Make daemon loop over coordinator connection to make it possible to create a system service awaiting coordinator connection by @haixuanTao in https://github.com/adora-rs/adora/pull/689
- Add translation example from chinese, french to english by @haixuanTao in https://github.com/adora-rs/adora/pull/681
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/690
- Fix macos 14 yolo error by @haixuanTao in https://github.com/adora-rs/adora/pull/696
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/692
- Publish rust project on pip to make it simpler to deploy adora node on different machine without requiring installing cargo by @haixuanTao in https://github.com/adora-rs/adora/pull/695
- Docs: README by @Radovenchyk in https://github.com/adora-rs/adora/pull/697
- Update README.md by @pucedoteth in https://github.com/adora-rs/adora/pull/705
- Bump rust toolchains 1.81 by @haixuanTao in https://github.com/adora-rs/adora/pull/707
- Make adora cli pip installable by @haixuanTao in https://github.com/adora-rs/adora/pull/706
- Add urdf visualization in rerun by @haixuanTao in https://github.com/adora-rs/adora/pull/704
- Fix child process receiving ctrl-c by setting own process group by @haixuanTao in https://github.com/adora-rs/adora/pull/712
- Move more types from `adora-core` to `adora-message` to avoid dependency by @phil-opp in https://github.com/adora-rs/adora/pull/711
- Implement `adora run` command by @phil-opp in https://github.com/adora-rs/adora/pull/703
- Adding Agilex Piper node, PyOrbbeckSDK node, Agilex UGV node by @haixuanTao in https://github.com/adora-rs/adora/pull/709
- Make the node hub CI/CD parallel for faster testing as well as having more granular integration control by @haixuanTao in https://github.com/adora-rs/adora/pull/710
- Add time series to adora rerun by @haixuanTao in https://github.com/adora-rs/adora/pull/713

## New Contributors

- @Ben-PH made their first contribution in https://github.com/adora-rs/adora/pull/685
- @Radovenchyk made their first contribution in https://github.com/adora-rs/adora/pull/697
- @pucedoteth made their first contribution in https://github.com/adora-rs/adora/pull/705

## v0.3.6 (2024-08-17)

## What's Changed

- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/579
- Don't wait for non-started dynamic nodes on stop by @phil-opp in https://github.com/adora-rs/adora/pull/583
- add a comment on read_adora_input_id by @XxChang in https://github.com/adora-rs/adora/pull/580
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/584
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/585
- Add domain unix socket supports by @XxChang in https://github.com/adora-rs/adora/pull/594
- Check build for cross-compiled targets on CI by @phil-opp in https://github.com/adora-rs/adora/pull/597
- Test pip release creation as part of normal CI by @phil-opp in https://github.com/adora-rs/adora/pull/596
- Add-armv7-musleabihf-prebuilt-release by @haixuanTao in https://github.com/adora-rs/adora/pull/578
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/602
- Delay dropping of `AdoraNode` in Python until all event data is freed by @phil-opp in https://github.com/adora-rs/adora/pull/601
- Add install script by @haixuanTao in https://github.com/adora-rs/adora/pull/600
- Nodes hub to store and reuse commonly used node by @haixuanTao in https://github.com/adora-rs/adora/pull/569
- Ros2-bridge action attempt by @starlitxiling in https://github.com/adora-rs/adora/pull/567
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/605
- Add a CI/CD for the node-hub by @haixuanTao in https://github.com/adora-rs/adora/pull/604
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/608
- Remove dynamic node from pending nodes before starting a dataflow by @haixuanTao in https://github.com/adora-rs/adora/pull/606
- Fix alignment of atomics in shared memory communication channel by @phil-opp in https://github.com/adora-rs/adora/pull/612
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/622
- Refactor: Move message definitions to `adora-message` crate by @phil-opp in https://github.com/adora-rs/adora/pull/613
- Update README.md by @heyong4725 in https://github.com/adora-rs/adora/pull/623
- Update Rust crate serde to v1.0.207 by @renovate in https://github.com/adora-rs/adora/pull/624
- fix clippy warnings by @Michael-J-Ward in https://github.com/adora-rs/adora/pull/626

## v0.3.5 (2024-07-03)

## What's Changed

- chore: Support RISCV64 by @LyonRust in https://github.com/adora-rs/adora/pull/505
- Json schemas for VSCode YAML Support by @haixuanTao in https://github.com/adora-rs/adora/pull/497
- Pretty Print Rust object when called from Python print by @haixuanTao in https://github.com/adora-rs/adora/pull/503
- Fix `Cargo.lock` by @phil-opp in https://github.com/adora-rs/adora/pull/506
- Use dependabot for automatic lockfile updates by @phil-opp in https://github.com/adora-rs/adora/pull/507
- Run cargo update by @phil-opp in https://github.com/adora-rs/adora/pull/508
- Allow top-level fields in node declaration by @phil-opp in https://github.com/adora-rs/adora/pull/478
- Configure Renovate by @renovate in https://github.com/adora-rs/adora/pull/509
- Make non-UTF8 stdout/stderr from nodes non-fatal by @phil-opp in https://github.com/adora-rs/adora/pull/510
- Make adora cli connect to remote coordinator by @Gege-Wang in https://github.com/adora-rs/adora/pull/513
- Provide help messages for CLI by @phil-opp in https://github.com/adora-rs/adora/pull/519
- Renovate: group all dependency updates in single PR by @phil-opp in https://github.com/adora-rs/adora/pull/524
- chore(deps): update dependencies by @renovate in https://github.com/adora-rs/adora/pull/529
- Improve coordinator port config by @phil-opp in https://github.com/adora-rs/adora/pull/520
- Fix some typos and add automatic typos check to CI by @EricLBuehler in https://github.com/adora-rs/adora/pull/539
- Update Pyo3 bounds by @Michael-J-Ward in https://github.com/adora-rs/adora/pull/472
- chore(deps): update dependencies by @renovate in https://github.com/adora-rs/adora/pull/543
- Small logging improvements by @phil-opp in https://github.com/adora-rs/adora/pull/537
- Refuse relative path for remote in coordinator by @XxChang in https://github.com/adora-rs/adora/pull/538
- chore(deps): update rust crate clap to v4.5.7 by @renovate in https://github.com/adora-rs/adora/pull/546
- Add `--quiet` flag to daemon and coordinator by @phil-opp in https://github.com/adora-rs/adora/pull/548
- Implement file-based logging in daemon and coordinator by @phil-opp in https://github.com/adora-rs/adora/pull/549
- Spawn daemon and coordinator in quiet mode on `adora up` by @phil-opp in https://github.com/adora-rs/adora/pull/550
- Run dynamic node by @haixuanTao in https://github.com/adora-rs/adora/pull/517
- Update adora new by @XxChang in https://github.com/adora-rs/adora/pull/553
- fix event_as_input bug by @XxChang in https://github.com/adora-rs/adora/pull/556
- Transform custom PyEvent into standard python dictionary for easier d… by @haixuanTao in https://github.com/adora-rs/adora/pull/557
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/558
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/560
- Update dependencies by @renovate in https://github.com/adora-rs/adora/pull/563
- Print only first node error and report more metadata in dataflow results by @phil-opp in https://github.com/adora-rs/adora/pull/552
- Make `adora start` attach by default, add `--detach` to opt-out by @phil-opp in https://github.com/adora-rs/adora/pull/561
- List failed and finished dataflows in `adora list` by @phil-opp in https://github.com/adora-rs/adora/pull/554
- Ignore-quicker-pending-drop-token by @haixuanTao in https://github.com/adora-rs/adora/pull/568
- Increasing grace duration to 2 seconds so that drop token get well returned in https://github.com/adora-rs/adora/pull/576

## New Contributors

- @LyonRust made their first contribution in https://github.com/adora-rs/adora/pull/505
- @renovate made their first contribution in https://github.com/adora-rs/adora/pull/509
- @Gege-Wang made their first contribution in https://github.com/adora-rs/adora/pull/513
- @EricLBuehler made their first contribution in https://github.com/adora-rs/adora/pull/539

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.4...v0.3.5

## v0.3.4 (2024-05-17)

## What's Changed

- Remove `cxx_build` call, which is no longer used by @phil-opp in https://github.com/adora-rs/adora/pull/470
- Update `ros2-client` to latest version by @phil-opp in https://github.com/adora-rs/adora/pull/457
- Configurable bind addrs by @Michael-J-Ward in https://github.com/adora-rs/adora/pull/471
- Simple warning fixes by @Michael-J-Ward in https://github.com/adora-rs/adora/pull/477
- Adding `adora-rerun` as a visualization tool by @haixuanTao in https://github.com/adora-rs/adora/pull/479
- Fix Clippy and RERUN_MEMORY_LIMIT env variable default by @haixuanTao in https://github.com/adora-rs/adora/pull/490
- Fix CI build errors by @phil-opp in https://github.com/adora-rs/adora/pull/491
- Use `resolver = 2` for in workspace in Rust template by @phil-opp in https://github.com/adora-rs/adora/pull/492
- Add grace duration and kill process by @haixuanTao in https://github.com/adora-rs/adora/pull/487
- Simplify parsing of `AMENT_PREFIX_PATH` by @haixuanTao in https://github.com/adora-rs/adora/pull/489
- Convert rust example to node by @Michael-J-Ward in https://github.com/adora-rs/adora/pull/494
- Adding python IDE typing by @haixuanTao in https://github.com/adora-rs/adora/pull/493
- Fix: Wait until adora daemon is connected to coordinator on `adora up` by @phil-opp in https://github.com/adora-rs/adora/pull/496

## New Contributors

- @Michael-J-Ward made their first contribution in https://github.com/adora-rs/adora/pull/471

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.3...v0.3.4

## v0.3.3 (2024-04-08)

## What's Changed

- Metrics refactoring by @haixuanTao in https://github.com/adora-rs/adora/pull/423
- Add ROS2 bridge support for C++ nodes by @phil-opp in https://github.com/adora-rs/adora/pull/425
- Provide function to create empty `CombinedEvents` stream by @phil-opp in https://github.com/adora-rs/adora/pull/432
- Expose ROS2 constants in generated bindings (Rust and C++) by @phil-opp in https://github.com/adora-rs/adora/pull/428
- Add option to send `stdout` as node/operator output by @haixuanTao in https://github.com/adora-rs/adora/pull/388
- Fix warning about `#pragma once` in main file by @phil-opp in https://github.com/adora-rs/adora/pull/433
- Send runs artefacts into a dedicated `out` folder by @haixuanTao in https://github.com/adora-rs/adora/pull/429
- Create README.md for cxx-ros2-example by @bobd988 in https://github.com/adora-rs/adora/pull/431
- Use Async Parquet Writer for `adora-record` by @haixuanTao in https://github.com/adora-rs/adora/pull/434
- Update mio to fix security vulnerability by @phil-opp in https://github.com/adora-rs/adora/pull/440
- Add initial support for calling ROS2 services from Rust nodes by @phil-opp in https://github.com/adora-rs/adora/pull/439
- Enable ROS2 service calls from C++ nodes by @phil-opp in https://github.com/adora-rs/adora/pull/441
- Use `Debug` formatting for eyre errors when returning to C++ by @phil-opp in https://github.com/adora-rs/adora/pull/450
- Fix out-of-tree builds in cmake example by @phil-opp in https://github.com/adora-rs/adora/pull/453
- Fix broken link in README by @mshr-h in https://github.com/adora-rs/adora/pull/462
- fix cargo run --example cmake-dataflow compile bugs by @XxChang in https://github.com/adora-rs/adora/pull/460
- Llm example by @haixuanTao in https://github.com/adora-rs/adora/pull/451
- Fix meter conflict by @haixuanTao in https://github.com/adora-rs/adora/pull/461
- Update README.md by @bobd988 in https://github.com/adora-rs/adora/pull/458
- Refactor `README` by @haixuanTao in https://github.com/adora-rs/adora/pull/463
- Specify conda env for Python Operators by @haixuanTao in https://github.com/adora-rs/adora/pull/468

## Minor

- Bump h2 from 0.3.24 to 0.3.26 by @dependabot in https://github.com/adora-rs/adora/pull/456
- Update `bat` dependency to v0.24 by @phil-opp in https://github.com/adora-rs/adora/pull/424

## New Contributors

- @bobd988 made their first contribution in https://github.com/adora-rs/adora/pull/431

* @mshr-h made their first contribution in https://github.com/adora-rs/adora/pull/462

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.2...v0.3.3

## v0.3.2 (2024-01-26)

## Features

- Wait until `DestroyResult` is sent before exiting adora-daemon by @phil-opp in https://github.com/adora-rs/adora/pull/413
- Reduce adora-rs to a single binary by @haixuanTao in https://github.com/adora-rs/adora/pull/410
- Rework python ROS2 (de)serialization using parsed ROS2 messages directly by @phil-opp in https://github.com/adora-rs/adora/pull/415
- Fix ros2 array bug by @haixuanTao in https://github.com/adora-rs/adora/pull/412
- Test ros2 type info by @haixuanTao in https://github.com/adora-rs/adora/pull/418
- Use forward slash as it is default way of defining ros2 topic by @haixuanTao in https://github.com/adora-rs/adora/pull/419

## Minor

- Bump h2 from 0.3.21 to 0.3.24 by @dependabot in https://github.com/adora-rs/adora/pull/414

## v0.3.1 (2024-01-09)

## Features

- Support legacy python by @haixuanTao in https://github.com/adora-rs/adora/pull/382
- Add an error catch in python `on_event` when using hot-reloading by @haixuanTao in https://github.com/adora-rs/adora/pull/372
- add cmake example by @XxChang in https://github.com/adora-rs/adora/pull/381
- Bump opentelemetry metrics to 0.21 by @haixuanTao in https://github.com/adora-rs/adora/pull/383
- Trace send_output as it can be a big source of overhead for large messages by @haixuanTao in https://github.com/adora-rs/adora/pull/384
- Adding a timeout method to not block indefinitely next event by @haixuanTao in https://github.com/adora-rs/adora/pull/386
- Adding `Vec<u8>` conversion by @haixuanTao in https://github.com/adora-rs/adora/pull/387
- Adora cli renaming by @haixuanTao in https://github.com/adora-rs/adora/pull/399
- Update `ros2-client` and `rustdds` dependencies to latest fork version by @phil-opp in https://github.com/adora-rs/adora/pull/397

## Fix

- Fix window path error by @haixuanTao in https://github.com/adora-rs/adora/pull/398
- Fix read error in C++ node input by @haixuanTao in https://github.com/adora-rs/adora/pull/406
- Bump unsafe-libyaml from 0.2.9 to 0.2.10 by @dependabot in https://github.com/adora-rs/adora/pull/400

## New Contributors

- @XxChang made their first contribution in https://github.com/adora-rs/adora/pull/381

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.3.0...v0.3.1

## v0.3.0 (2023-11-01)

## Features

- Rust node API typed using arrow by @phil-opp in https://github.com/adora-rs/adora/pull/353
- Adora record by @haixuanTao in https://github.com/adora-rs/adora/pull/365
- beautify graph visualisation by @haixuanTao in https://github.com/adora-rs/adora/pull/370
- Remove `Ros2Value` encapsulation of `ArrayData` by @haixuanTao in https://github.com/adora-rs/adora/pull/359
- Refactor python typing by @haixuanTao in https://github.com/adora-rs/adora/pull/369
- Update README discord link by @Felixhuangsiling in https://github.com/adora-rs/adora/pull/361

### Other

- Update `rustix` v0.38 dependency by @phil-opp in https://github.com/adora-rs/adora/pull/366
- Bump rustix from 0.37.24 to 0.37.25 by @dependabot in https://github.com/adora-rs/adora/pull/364
- Bump quinn-proto from 0.9.3 to 0.9.5 by @dependabot in https://github.com/adora-rs/adora/pull/357
- Bump webpki from 0.22.1 to 0.22.2 by @dependabot in https://github.com/adora-rs/adora/pull/358
- Update README discord link by @Felixhuangsiling in https://github.com/adora-rs/adora/pull/361

## New Contributors

- @Felixhuangsiling made their first contribution in https://github.com/adora-rs/adora/pull/361

## v0.2.6 (2023-09-14)

- Update dependencies to fix some security advisories by @phil-opp in https://github.com/adora-rs/adora/pull/354
  - Fixes `cargo install adora-daemon`

## v0.2.5 (2023-09-06)

### Features

- Use cargo instead of git in Rust `Cargo.toml` template by @haixuanTao in https://github.com/adora-rs/adora/pull/326
- Use read_line instead of next_line in stderr by @haixuanTao in https://github.com/adora-rs/adora/pull/325
- Add a `rust-ros2-dataflow` example using the adora-ros2-bridge by @phil-opp in https://github.com/adora-rs/adora/pull/324
- Removing patchelf by @haixuanTao in https://github.com/adora-rs/adora/pull/333
- Improving python example readability by @haixuanTao in https://github.com/adora-rs/adora/pull/334
- Use `serde_bytes` to serialize `Vec<u8>` by @haixuanTao in https://github.com/adora-rs/adora/pull/336
- Adding support for `Arrow List(*)` for Python by @haixuanTao in https://github.com/adora-rs/adora/pull/337
- Bump rustls-webpki from 0.100.1 to 0.100.2 by @dependabot in https://github.com/adora-rs/adora/pull/340
- Add support for event stream merging for Python node API by @phil-opp in https://github.com/adora-rs/adora/pull/339
- Merge `adora-ros2-bridge` by @phil-opp in https://github.com/adora-rs/adora/pull/341
- Update dependencies by @phil-opp in https://github.com/adora-rs/adora/pull/345
- Add support for arbitrary Arrow types in Python API by @phil-opp in https://github.com/adora-rs/adora/pull/343
- Use typed inputs in Python ROS2 example by @phil-opp in https://github.com/adora-rs/adora/pull/346
- Use struct type instead of array for ros2 messages by @haixuanTao in https://github.com/adora-rs/adora/pull/349

### Other

- Add Discord :speech_balloon: by @haixuanTao in https://github.com/adora-rs/adora/pull/348
- Small refactoring by @haixuanTao in https://github.com/adora-rs/adora/pull/342

## v0.2.4 (2023-07-18)

### Features

- Return dataflow result to CLI on `adora stop` by @phil-opp in https://github.com/adora-rs/adora/pull/300
- Make dataflow descriptor available to Python nodes and operators by @phil-opp in https://github.com/adora-rs/adora/pull/301
- Create a `CONTRIBUTING.md` guide by @phil-opp in https://github.com/adora-rs/adora/pull/307
- Distribute prebuilt arm macos adora-rs by @haixuanTao in https://github.com/adora-rs/adora/pull/308

### Other

- Fix the typos and add adora code branch by @meua in https://github.com/adora-rs/adora/pull/290
- For consistency with other examples, modify python -> python3 by @meua in https://github.com/adora-rs/adora/pull/299
- Add timestamps generated by hybrid logical clocks to all sent events by @phil-opp in https://github.com/adora-rs/adora/pull/302
- Don't recompile the `adora-operator-api-c` crate on every build/run by @phil-opp in https://github.com/adora-rs/adora/pull/304
- Remove deprecated `proc_macros` feature from `safer-ffi` dependency by @phil-opp in https://github.com/adora-rs/adora/pull/305
- Update to Rust v1.70 by @phil-opp in https://github.com/adora-rs/adora/pull/303
- Fix issue with not finding a custom nodes path by @haixuanTao in https://github.com/adora-rs/adora/pull/315
- Implement `Stream` for `EventStream` by @phil-opp in https://github.com/adora-rs/adora/pull/309
- Replace unmaintained `atty` crate with `std::io::IsTerminal` by @phil-opp in https://github.com/adora-rs/adora/pull/318

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.2.3...v0.2.4

## v0.2.3 (2023-05-24)

## What's Changed

- Check that coordinator, daemon, and node versions match by @phil-opp in https://github.com/adora-rs/adora/pull/245
- Share events to Python without copying via `arrow` crate by @phil-opp in https://github.com/adora-rs/adora/pull/228
- Upgrading the operator example to use `adora-arrow` by @haixuanTao in https://github.com/adora-rs/adora/pull/251
- [Python] Show node name in process and put Traceback before the actual Error for more natural error by @haixuanTao in https://github.com/adora-rs/adora/pull/255
- CLI: Improve error messages when coordinator is not running by @phil-opp in https://github.com/adora-rs/adora/pull/254
- Integrate `adora-runtime` into `adora-daemon` by @phil-opp in https://github.com/adora-rs/adora/pull/257
- Filter default log level at `warn` for `tokio::tracing` by @haixuanTao in https://github.com/adora-rs/adora/pull/269
- Make log level filtering be `WARN` or below by @haixuanTao in https://github.com/adora-rs/adora/pull/274
- Add support for distributed deployments with multiple daemons by @phil-opp in https://github.com/adora-rs/adora/pull/256
- Provide a way to access logs through the CLI by @haixuanTao in https://github.com/adora-rs/adora/pull/259
- Handle node errors during initialization phase by @phil-opp in https://github.com/adora-rs/adora/pull/275
- Replace watchdog by asynchronous heartbeat messages by @phil-opp in https://github.com/adora-rs/adora/pull/278
- Remove pyo3 in runtime and daemon as it generates `libpython` depende… by @haixuanTao in https://github.com/adora-rs/adora/pull/281
- Release v0.2.3 with aarch64 support by @haixuanTao in https://github.com/adora-rs/adora/pull/279

## Fix

- Fix yolov5 dependency issue by @haixuanTao in https://github.com/adora-rs/adora/pull/291
- To solve this bug https://github.com/adora-rs/adora/issues/283, unify t… by @meua in https://github.com/adora-rs/adora/pull/285
- Fix: Don't try to create two global tracing subscribers when using bundled runtime by @phil-opp in https://github.com/adora-rs/adora/pull/277
- CI: Increase timeout for 'build CLI and binaries' step by @phil-opp in https://github.com/adora-rs/adora/pull/282

## Other

- Update `pyo3` to `v0.18` by @phil-opp in https://github.com/adora-rs/adora/pull/246
- Bump h2 from 0.3.13 to 0.3.17 by @dependabot in https://github.com/adora-rs/adora/pull/249
- Add automatic issue labeler to organize opened issues by @haixuanTao in https://github.com/adora-rs/adora/pull/265
- Allow the issue labeler to write issues by @phil-opp in https://github.com/adora-rs/adora/pull/272
- Add a support matrix with planned feature to clarify adora status by @haixuanTao in https://github.com/adora-rs/adora/pull/264

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.2.2...v0.2.3

## v0.2.2 (2023-04-01)

### Features

- Make queue length configurable through the dataflow file by @phil-opp in https://github.com/adora-rs/adora/pull/231
- Hot reloading Python Operator by @haixuanTao in https://github.com/adora-rs/adora/pull/239
- Synchronize node and operator start by @phil-opp in https://github.com/adora-rs/adora/pull/236
- Add opentelemetry capability at runtime instead of compile time by @haixuanTao in https://github.com/adora-rs/adora/pull/234

### Others

- Wait on events and messages simultaneously to prevent queue buildup by @phil-opp in https://github.com/adora-rs/adora/pull/235
- Fix looping in daemon listener loop by @phil-opp in https://github.com/adora-rs/adora/pull/244
- Validate shell command as source and url source by @haixuanTao in https://github.com/adora-rs/adora/pull/243
- Push error into the `init_done` channel for debugging context by @haixuanTao in https://github.com/adora-rs/adora/pull/238
- Option communication config by @haixuanTao in https://github.com/adora-rs/adora/pull/241
- Validate yaml when reading by @haixuanTao in https://github.com/adora-rs/adora/pull/237

**Full Changelog**: https://github.com/adora-rs/adora/compare/v0.2.1...v0.2.2

## v0.2.1 (2023-03-22)

### Features

- [Make adora-rs publishable on crates.io](https://github.com/adora-rs/adora/pull/211)

### Fixes

- [Avoid blocking the daemon main loop by using unbounded queue](https://github.com/adora-rs/adora/pull/230)
- [Inject YAML declared env variable into the runtime](https://github.com/adora-rs/adora/pull/227)
- [Use rustls instead of system SSL implementation](https://github.com/adora-rs/adora/pull/216)

### Other

- [Refactor python error](https://github.com/adora-rs/adora/pull/229)
- [The first letter of rust should be lowercase in the command](https://github.com/adora-rs/adora/pull/226)
- [Add documentation to the cli within the helper mode](https://github.com/adora-rs/adora/pull/225)
- [Update to safer-ffi v0.1.0-rc1](https://github.com/adora-rs/adora/pull/218)
- [remove unused variable: data_bytes](https://github.com/adora-rs/adora/pull/215)
- [Clean up: Remove workspace path](https://github.com/adora-rs/adora/pull/210)
- [Decouple opentelemetry from tracing](https://github.com/adora-rs/adora/pull/222)
- [Remove zenoh dependency from adora node API to speed up build](https://github.com/adora-rs/adora/pull/220)
- [Update to Rust v1.68](https://github.com/adora-rs/adora/pull/221)
- [Deny unknown fields to avoid typos](https://github.com/adora-rs/adora/pull/223)
- [Add an internal cli argument to create template with path dependencies](https://github.com/adora-rs/adora/pull/212)

## v0.2.0 (2023-03-14)

### Breaking

- [Redesign: Create a `adora-daemon` as a communication broker](https://github.com/adora-rs/adora/pull/162)
  - New `adora-daemon` executable that acts as a communication hub for all local nodes
  - Large messages are passed through shared memory without any copying
  - [Replaces the previous `iceoryx` communication layer](https://github.com/adora-rs/adora/pull/201)
  - Small API change: Nodes and operators now receive _events_ instead of just inputs
    - Inputs are one type of event
    - Other supported events: `InputClosed` when an input stream is closed and `Stop` when the user stops the dataflow (e.g. through the CLI)

### Features

- Better Error handling when operator fails
- [Send small messages directly without shared memory](https://github.com/adora-rs/adora/pull/193)
- [Send all queued incoming events at once on `NextEvent` request](https://github.com/adora-rs/adora/pull/194)
- [Don't send replies for `SendMessage` requests when using TCP](https://github.com/adora-rs/adora/pull/195)
- [Allocate shared memory in nodes to improve throughput](https://github.com/adora-rs/adora/pull/200)

### Fixes

- [Manage node failure: Await all nodes to finish before marking dataflow as finished](https://github.com/adora-rs/adora/pull/183)

### Other

- [Use `AdoraStatus` from adora library in template](https://github.com/adora-rs/adora/pull/182)
- [Simplify: Replace `library_filename` function with `format!` call](https://github.com/adora-rs/adora/pull/191)
- [Refactor Rust node API implementation](https://github.com/adora-rs/adora/pull/196)
- [Remove code duplicate for tracing subscriber and use env variable to manage log level.](https://github.com/adora-rs/adora/pull/197)
- [Add daemon to the release archive](https://github.com/adora-rs/adora/pull/199)
- [Remove `remove_dir_all` from `Cargo.lock`as it is vulnerable to a race condition according to dependabot](https://github.com/adora-rs/adora/pull/202)
- [Update the documentation to the new daemon format](https://github.com/adora-rs/adora/pull/198)
- [Removing legacy `libacl` which was required by Iceoryx](https://github.com/adora-rs/adora/pull/205)
- [Remove unimplemented CLI arguments for now](https://github.com/adora-rs/adora/pull/207)
- [Update zenoh to remove git dependencies](https://github.com/adora-rs/adora/pull/203)
- [Fix cli template to new daemon API](https://github.com/adora-rs/adora/pull/204)
- [Cleanup warnings](https://github.com/adora-rs/adora/pull/208)
- Dependency updates

## v0.1.3 (2023-01-18)

- Package `AdoraStatus` into adora python package: https://github.com/adora-rs/adora/pull/172
- Force removal of Pyo3 Object to avoid memory leak: https://github.com/adora-rs/adora/pull/168
- Bump tokio from 1.21.2 to 1.23.1: https://github.com/adora-rs/adora/pull/171
- Create a changelog file: https://github.com/adora-rs/adora/pull/174

## v0.1.2 (2022-12-15)

- Fix infinite loop in the coordinator: https://github.com/adora-rs/adora/pull/155
- Simplify the release process: https://github.com/adora-rs/adora/pull/157
- Use generic linux distribution: https://github.com/adora-rs/adora/pull/159

## v0.1.1 (2022-12-05)

This release contains fixes for:

- Python linking using pypi release but also a redesigned python thread model within the runtime to avoid deadlock of the `GIL`. This also fix an issue with `patchelf`.
- A deployment separation for `ubuntu` as the `20.04` version of `adora` and `22.04` version of adora are non-compatible.
- A better tagging of api for `adora` Rust API.

## v0.1.0 (2022-11-15)

This is our first release of `adora-rs`!

The current release includes:

- `adora-cli` which enables creating, starting and stopping dataflow.
- `adora-coordinator` which is our control plane.
- `adora-runtime` which is manage the runtime of operators.
- `custom-nodes` API which enables bridges from different languages.
