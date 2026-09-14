# 2026-07-31 challenge task two white-border solver validation

## Change scope

- Replaced the task-3 playing-card solver path with a white-border outer-frame lock.
- Added per-edge white-border confidence in `PuzzleSolverCore`; task 3 now keeps white edges exposed and only merges non-white cut edges.
- Added ambiguity scoring for seam continuity, center symmetry, diagonal corner specialness, and non-character-corner whiteness.
- Added task-3 YAML parameters and two solver unit tests for white-border locking.

## Backups

- Windows backup:
  `D:\CodexFolder\JetsonNano\Copyfiles\26E_vision\backups\challenge_two_white_border_solver\20260731_110659`
- Jetson backup:
  `/home/jetson/ProjectsByMonthWU/VisionJetson/backups/26E_vision/challenge_two_white_border_solver/20260731_110659`

## Deployed files

All selected files were deployed from Windows authority to:
`/home/jetson/ProjectsByMonthWU/VisionJetson/Copyfiles/26E_vision`.

Remote post-deploy SHA256:

```text
14512c1bb8a64b5c328c0497ce65cc494629da9d9be6056d7f145348d9bf1917  AGENTS.md
5bb5e5c4af7fa6e916d8d5fe40097a88b5576945d7c1a32c1225d440ea32d4f2  README.md
ecc6cd865e8b5200030054c40ee251e1fa24656dec7ae0ddfd39b83a8ee1ccc6  src/puzzle_solver_node/include/puzzle_solver_node/puzzle_solver_core.hpp
d0f3e468f1d6581a2404fb8714c25e013130b48dd292b96938412f13a1c4f18b  src/puzzle_solver_node/src/puzzle_solver_core.cpp
c322cbc1acd3e0bb211c80af7e15694082b8d2c5b1378810cde4718e9d6724b3  src/puzzle_solver_node/src/puzzle_solver_node.cpp
e2fe91d401ca5a0a315a2c110be29d6cd8b51bd6e0a0db8393a58b9b4187b172  src/puzzle_solver_node/test/test_puzzle_solver_core.cpp
9832e33bf165d86c9a83b1f42cb18ac12461ee5319c1dcebacc7e921e9b878a7  docs/challenge_task_two.md
0490ed00328ea633ec8f2fa9928c7ec13630e6bd6f8aa26e041b5ba17cd04115  src/vision_bringup/config/vision_system.yaml
```

## Jetson validation

Identity:

```text
aarch64
monthwu-jetson
jetson
```

Build:

```text
colcon build --packages-select puzzle_solver_node vision_bringup --event-handlers console_direct+
Summary: 2 packages finished
```

Tests:

```text
colcon test --packages-select puzzle_solver_node --event-handlers console_direct+
test_puzzle_solver_core: 16 tests passed
colcon test-result --all: 25 tests, 0 errors, 0 failures, 0 skipped
```

Runtime smoke:

```text
ros2 run puzzle_solver_node puzzle_solver_node --ros-args -r __node:=puzzle_solver_node_white_border_smoke --params-file install/vision_bringup/share/vision_bringup/config/vision_system.yaml
ros2 param get /puzzle_solver_node_white_border_smoke challenge_two_white_border_fast_solver_enabled
Boolean value is: True
ros2 param get /puzzle_solver_node_white_border_smoke challenge_two_center_symmetry_direct_threshold
Double value is: 0.9
```

The temporary smoke process group was terminated and did not remain. Pre-existing nodes were left running and were not stopped.

## Remaining acceptance boundary

This validates source synchronization, ARM64 build, solver unit behavior, and parameter loading. It does not validate real playing-card images, camera calibration quality, UART parsing, MCU motion, or physical placement accuracy.

## Rollback

Restore files from the Windows backup to the Windows authority, restore files from the Jetson backup to the Jetson deployment mirror, then rebuild `puzzle_solver_node` and `vision_bringup`. Delete this validation report if rolling back the documentation addition.
