> **Historical / superseded — retained in place.** Current owners: [ROS guide](../ros2.md), [command/API contract](../thin-api.md), [mapping/depth guide](../../src/z70_mapping/README.md).
> This is the exact pre-cutover index snapshot, not a current inventory or install guide. Counts, old `ros2` paths, `depth=false` and tracked-venv state are dated observations. Plans and commands below are not present-day instructions.

JSON inventories and source assets are preserved unchanged. Model identifiers do not prove cached or tracked weights exist; old snapshot evidence must not be regenerated from HEAD.

## Historical record

# Offline tracked-asset inventory

## Scope and reproducible boundary

This is a **Git index inventory**, not a disk/cache inventory or a cleanup authorization. Snapshot base: `f0484c2d594ed035240210b5795368272fa6c45f`. The sorted stage-0 `git ls-files --stage -z` byte stream has SHA-256 `98f446eca8152dfaa4d76bfacd35c34ea844c4ef0d2823a53ec0fa5eb3b36546`.

[`tracked-assets.json`](tracked-assets.json) contains **every one of the 3,375 tracked paths**, with exact index blob size, Git object ID/mode, categories, proposed disposition and a source-backed `condition_ref`. No glob is a substitute for its `assets` list. Sizes describe the indexed version, **not concurrent worktree modifications**, LFS payloads, filesystem allocation or deduplicated repository storage. Symlink sizes describe stored link text. The two new audit documents are not in this index snapshot.

Only Git metadata, pointer-sized index blobs and selected source/documentation/package metadata were inspected. Large dependency binary contents were not scanned. No hardware, camera, service, Docker, network, ignored-cache discovery, `.env`, `.local`, token or private-state access was performed. No files were deleted, moved, staged or committed. Existing concurrent modifications were left untouched.

## Verified counts

| Category | Tracked paths | Sum of index blob bytes |
|---|---:|---:|
| Entire index | 3,375 | 264,456,836 |
| Tracked environment: `calibration/arm-validation/.venv/` | 2,581 | 255,980,372 |
| ↳ Dependency binaries | 255 | 214,102,144 |
| ↳ Dependency data, by documented extension rule | 9 | 3,983,474 |
| ↳ Python `.pth` path hook | 1 | 152 |
| ↳ Environment symlinks | 3 | 65 |
| ↳ Remaining environment support files | 2,313 | 37,894,537 |
| ROS installed model resources | 8 | 345,313 |
| ROS model generator source | 1 | 5,947 |
| Active calibration data outside ROS package | 3 | 15,347 |
| Runtime calibration helper outside ROS package | 1 | 3,315 |
| ROS depth runtime source | 3 | 11,096 |
| Experimental depth consumer/docs | 2 | 13,627 |
| Experimental depth result evidence | 3 | 2,713 |
| Other calibration source | 79 | 464,587 |
| Other calibration evidence | 349 | 5,174,713 |
| Remaining source/configuration/documentation | 345 | 2,439,806 |

Environment subrows overlap the environment parent row; all other category rows are disjoint. Dependency binaries comprise **140 `.so`, 103 `.dylib`, 10 `.exe`, and 2 `.a` paths**, all within that tracked environment. These are dependency/native-library artifacts, not neural weights. Dependency-data classification is extension-based (`.npz`, `.npy`, `.pkl`, `.pickle`, `.mat`, `.gz`, `.bz2`, `.xz`, `.fits`, `.wav`, `.dat`, `.sav`, `.mtx`, `.arff`); it is not a binary-format inspection or an exhaustive MIME claim. All environment files, including otherwise unrecognized packaged data, remain covered by `tracked_venv`.

### Model weights and LFS

- **Confirmed tracked neural weight files: 0.** No tracked `.pt`, `.ckpt`, `.onnx`, `.safetensors`, `.bin`, `.h5`, `.hdf5`, `.tflite`, `.pb`, `.gguf` or `.weights` paths were found. The sole `.pth` candidate is `calibration/arm-validation/.venv/lib/python3.9/site-packages/distutils-precedence.pth` (152 bytes): its indexed source installs the setuptools/distutils shim; it is not a PyTorch checkpoint.
- **Standard LFS pointer paths found: 0.** Pointer detection checked Git-index blobs of at most 1,024 bytes for the LFS v1 header and `oid sha256`/`size` fields. No private paths needed exclusion. No `.gitattributes` is tracked. This is an indexed-pointer check, not a claim about LFS objects elsewhere or repository history; no LFS fetch/smudge was run.
- **Current external cached weights: unknown, intentionally not inspected.** The repository contains model identifiers and inference consumers, but those do not establish the presence, version, size or location of an actual cached checkpoint. The documented `~/.venvs/roboclaw-depth` environment is external; it is not the tracked arm-validation environment. Do not treat a code model name, metric `model.json`, URDF, or validation result filename as weights.

The JSON provides explicit `model_weights.extension_candidate_paths`, `model_weights.confirmed_tracked_weight_paths`, and `lfs_detection.pointer_paths` lists, including empty lists where verified.

## Depth Pro / DA2 Base experiments versus ROS DA2 Small

| Boundary | Exact consumers and evidence | Decision |
|---|---|---|
| Offline single-frame prototype | `calibration/pointcloud/pointcloud.py`; `calibration/pointcloud/README.md` | Keep or extract as an experiment; not an installed ROS inference entry point. |
| Experimental results | `calibration/pointcloud/out/pointcloud_validation_da2-indoor-small.json`; `calibration/pointcloud/out/pointcloud_validation_da2-indoor-base.json`; `calibration/pointcloud/out/pointcloud_validation_depth-pro.json` | Three tracked result records, not model weights. Preserve provenance before any separately approved retirement. |
| ROS optional depth layer | `ros2/src/z70_mapping/z70_mapping/depth_node.py`; `ros2/src/z70_mapping/z70_mapping/depth_worker.py`; `ros2/src/z70_mapping/z70_mapping/depth_geometry.py` | Keep with optional-depth launch/package integration. |

Source-backed distinctions:

1. `calibration/pointcloud/pointcloud.py:32–45,93–104` explicitly maps CLI choices to `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`, `depth-anything/Depth-Anything-V2-Metric-Indoor-Base-hf` and `apple/DepthPro-hf`. It uses Transformers `from_pretrained` **without** `local_files_only=True`; running the prototype is not guaranteed offline. This audit did not execute it.
2. The prototype loads `calibration/front-camera/front_camera.json` and `calibration/front-camera/out/detections.json` (`pointcloud.py:100,150`). It writes model-keyed validation JSON and generated PLY/media (`:186–193` and following). The README documents comparisons and the external inference environment (`README.md:7–11,20–39`). These are evidence of intended/reported experiments, not proof that weights are currently installed.
3. The ROS worker defaults specifically to **DA2 Metric Indoor Small**, uses `local_files_only=True` for both processor and model, and defaults to MPS (`depth_worker.py:27–43`). Its CLI permits `--model`, but the ROS parent passes only calibration/device, **not a model override** (`depth_node.py:25–40`). Therefore the shipped parent/worker path uses Small; it does not select Base or Depth Pro.
4. ROS uses an external interpreter at `~/.venvs/roboclaw-depth/bin/python` by default, subscribes to the existing compressed JPEG topic and spawns a persistent worker (`depth_node.py:25–40`). The optional feature is disabled by default in `ros2/src/z70_bringup/launch/bringup.launch.py:35–41`; `ros2/src/z70_mapping/setup.py:7–8` exposes `depth_cloud`. This is source analysis, not live availability verification.
5. `depth_geometry.py:1–3` acknowledges adaptation from the prototype. Removing the experimental CLI is not a reason to remove the ROS geometry worker or shared inference environment. Retiring Base/Depth Pro would require a separate approved decision about experimental reproducibility; no such retirement is performed here.

## Keep calibration, runtime inputs and install resources distinct

### Active runtime data outside package installation

Keep these exact paths:

- `calibration/front-camera/front_camera.json`
- `calibration/front-camera/z70_front_camera_info.yaml`
- `calibration/chassis/online.json`
- `calibration/front-camera/floor_geometry.py` — dynamically imported runtime helper, not disposable analysis-only code.

`bringup.launch.py:19–29,49–54` wires front-camera JSON/YAML into runtime nodes and the chassis calibrator's output path. `ros2/src/z70_manipulation/z70_manipulation/perception.py:11–25` dynamically loads the floor helper and front calibration, and consumes installed mount/gripper calibration. `z70_manipulation/node.py:136` reads chassis gains. `calibration/front-camera/export.py` produces the front JSON, but a generated runtime input is not thereby disposable. An extraction must package the required data/helper, rewrite repository-relative consumers and validate the new resource resolution first.

### Installed ROS model resources: eight paths

`ros2/src/z70_description/setup.py:9–15` installs these resources:

- `ros2/src/z70_description/config/mount_calibration.yaml`
- `ros2/src/z70_description/config/z70_gripper_camera_info.yaml`
- `ros2/src/z70_description/meshes/pincher_finger_left.stl`
- `ros2/src/z70_description/meshes/pincher_finger_right.stl`
- `ros2/src/z70_description/meshes/pincher_lens.stl`
- `ros2/src/z70_description/meshes/pincher_shell.stl`
- `ros2/src/z70_description/rviz/z70.rviz`
- `ros2/src/z70_description/urdf/z70.urdf.xacro`

Keep `ros2/src/z70_description/meshes/generate_pincher.py` separately as the generator source; it is **not** installed by the `meshes/*.stl` glob. Xacro loads mount calibration and references STL package URIs (`z70.urdf.xacro:25,122,138,158`). A mesh or calibration filename alone must not trigger a deletion rule.

The three tracked URDF snapshots under `calibration/physical-measurements/20261015-pincher-underside/model-update/` — `after.urdf`, `before.urdf`, `live-description.urdf` — are **calibration/model-update evidence**, not the installed live Xacro and not neural weights. Their exact indexed metadata remains in the JSON.

### Calibration evidence can have regression consumers

Do not delete `calibration/` or its `out/` trees wholesale. Examples from source, without reproducing captured evidence contents:

- `ros2/src/z70_description/test/test_provisional_model.py:18–19,74–79` consumes the provisional-model JSON and proposed lens calibration evidence.
- `ros2/src/z70_description/test/test_front_floor_tf.py:17–21,36–41` consumes front registration and an archived prior calibration; it checks the prior file's digest.
- Installed configuration retains evidence-path provenance (`config/mount_calibration.yaml:80–151`, `config/z70_gripper_camera_info.yaml:65`). Provenance references are not the same as runtime file reads, but still need preservation or an intentional rewrite.

Thus `calibration_evidence` and `calibration_source` default to **keep pending per-file consumer audit**, not “safe to delete.” This inventory identifies the boundary and known consumers; it does not assert an exhaustive dependency graph for every historical record.

## Tracked environment: extract before considering removal

Exactly **2,581 indexed paths** are under `calibration/arm-validation/.venv/`. Exact native-binary paths and every support file are enumerated in JSON, not hidden behind directory-level estimates. Largest indexed blobs include:

| Exact tracked path | Bytes |
|---|---:|
| `calibration/arm-validation/.venv/lib/python3.9/site-packages/cv2/cv2.abi3.so` | 60,926,016 |
| `calibration/arm-validation/.venv/lib/python3.9/site-packages/scipy/.dylibs/libopenblas.0.dylib` | 19,738,544 |
| `calibration/arm-validation/.venv/lib/python3.9/site-packages/cv2/.dylibs/libavcodec.61.19.101.dylib` | 10,298,272 |

Indexed distribution `METADATA` records identify NumPy **2.0.2**, opencv-contrib-python **5.0.0.93**, SciPy **1.13.1**, pip **21.2.4**, setuptools **58.0.4**. These are tracked-environment metadata versions, not import/runtime compatibility tests. Matching `.dist-info/METADATA`, `.dist-info/RECORD`, license and support paths are included in the inventory. Python 3.9/Darwin-specific extension names and interpreter symlinks are present; no symlink target was executed or followed.

Known consumers precede any disposal decision:

- `calibration/arm-validation/analyze.py:1–15` documents `.venv/bin/python` and imports OpenCV/NumPy, using front calibration.
- `calibration/arm-validation/m3-m4-session/report.py:1–12` imports SciPy's `least_squares`; `calibration/arm-validation/M3_M4_RESULTS.md:12` documents reproduction using `.venv/bin/python m3-m4-session/report.py`.
- `.gitignore` has no general `.venv` exclusion. Adding an ignore rule alone would not untrack these already-indexed files.

**Conditional recommendation, not authorization:** extract a reproducible Python/platform/dependency recipe, required package metadata/licenses and offline analysis provenance. Reproduce both analysis/report workflows in a replacement environment and update their documented interpreter paths. Only then consider separately authorized untracking/removal of the environment. Keep calibration inputs/results and the external ROS depth environment out of that deletion scope. No replacement environment was built or tested in this documentation-only phase.

## Verification and use of the JSON

Inventory construction used `git ls-files --stage -z` for membership and `git cat-file --batch-check` for blob lengths, not a recursive working-tree scan. Standard pointer detection read only pointer-sized blobs. Validation parsed the saved JSON, compared its full unique path set with a fresh index listing, matched the index-listing digest, and recomputed category counts/byte sums. No repository test suite or live validation was run.

Each asset's `condition_ref` resolves to `conditions`, where keep/extract/delete prerequisites and exact source references are recorded. Dispositions are recommendations only. To obtain exact subsets without relying on this prose, for example with `jq`:

```sh
jq -r '.assets[] | select(.categories | index("tracked_venv")) | .path' docs/migration/tracked-assets.json
jq -r '.assets[] | select(.categories | index("dependency_binary")) | [.path,.size_bytes] | @tsv' docs/migration/tracked-assets.json
jq '.lfs_detection.pointer_paths, .model_weights' docs/migration/tracked-assets.json
```

Any later migration must repeat the index snapshot and consumer audit if membership or relevant sources change. Existing concurrent worktree edits mean indexed sizes and checked-out sizes may already differ; this document deliberately makes no claim otherwise.
