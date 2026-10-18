> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md).
> The record below preserves its original observations, source pins and reported results. Its plans, commands, permissions and deployment statements are not present-day instructions or reusable live authorization.

Results below are reported phase-1 provenance. build_path_map.py, verify_phase1.py and npm-test.txt are unavailable at their historical docs/migration paths; no original output was reconstructed. These are not current maintenance commands.

**Historical outputs unavailable in this checkout:** `docs/migration/build_path_map.py`, `docs/migration/npm-test.txt`, `docs/migration/verify_phase1.py`. Results citing them below are reported history only.

## Historical record

# Phase-1 verification

This records offline documentation checks, not runtime/robot qualification.

Completed during preparation:
- `python3 docs/migration/build_path_map.py`: generated one design mapping per indexed path; destination collision assertion passed.
- Runtime boundary spot checks: dispatch/retry semantics and pre-call sentAt (`src/service.ts:193–249`), outcome types (`src/rpc-outcome.ts:1–18`), production arm-constant importers, bridge receipt stamps and folded display placeholders (`ros2/src/z70_bridge/z70_bridge/bridge.py:29–47,138–174`), current HTTP executor and pulse bounds (`ros2/src/z70_manipulation/z70_manipulation/executor.py:22–47,154–182`), mapping refresh/snapshot caveats and launch defaults were read directly.

Final results:

| Check actually executed | Result |
|---|---|
| `python3 docs/migration/verify_phase1.py` | PASS: 3,375 exact indexed assets and mapping rows; 264,456,836 summed indexed blob bytes; 794 unique non-retired destinations; 0 standard LFS pointers; 1 weight-extension candidate (setuptools `.pth` hook), 0 identified tracked neural checkpoints; 260 source citation path/ranges and 8 local links checked |
| `npm test` | PASS: 370 tests, 370 passed, 0 failed/cancelled/skipped/todo; captured output in `npm-test.txt` |
| `npm run typecheck` | PASS for backend and frontend |
| `npm run build` | PASS: typecheck + Vite client production build, 20 modules transformed |
| `git diff --check` | PASS for tracked changes; inventory verifier also validates newly written JSON |
| Launch Python syntax | PASS via `ast.parse` in documentation verifier; launch was not executed |
| ROS suite / generated target IDL | Not run/generated in this documentation-only phase; no ROS algorithm or interface implementation changed. Phase 2 must compile/serialize generated interfaces and run the whole ROS suite. |


The root/src runtime and ROS algorithms are unchanged; the launch edit changes its descriptive docstring only. No deployed state or hardware outcome is claimed. Concurrent calibration/geometry/manipulation changes are not part of this phase's verification scope. Test success does not certify those unrelated edits or the future contracts' physical behavior. No files were staged or committed.
