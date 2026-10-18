# Phase 3 — installed resources and durable calibration

## Binding CalibrationState schema (for interface owner)

Add `z70_interfaces/msg/CalibrationState.msg` exactly as below. All consumers of
`/z70/chassis_calibration` switch together from String to this type. Publisher
and subscribers use RELIABLE TRANSIENT_LOCAL depth 1. Values are SI; gains are
dimensionless. Unknown numeric values are zero with their known flag false, not
NaN or fabricated certainty. `camera_x_ready` retains the existing fit gate;
fit uncertainty is not absolute accuracy. Calibration is ESTIMATED, never OBSERVED.

```msg
std_msgs/Header header
z70_interfaces/EvidenceStamp evidence
z70_interfaces/Validity validity
string<=128 calibration_id
uint32 sample_count
uint32 skipped_count
bool yaw_positive_known
float64 yaw_positive_gain
bool yaw_positive_sigma_known
float64 yaw_positive_sigma
uint32 yaw_positive_count
bool yaw_negative_known
float64 yaw_negative_gain
bool yaw_negative_sigma_known
float64 yaw_negative_sigma
uint32 yaw_negative_count
bool forward_known
float64 forward_gain
bool forward_sigma_known
float64 forward_sigma
uint32 forward_count
bool camera_x_known
float64 camera_x_m
bool camera_x_sigma_known
float64 camera_x_sigma_m
uint32 camera_x_count
bool camera_x_residual_known
float64 camera_x_residual_rms_m
bool camera_x_ready
string<=256 model
```

`header.stamp`/evidence source time identify persisted estimator update, not timer
publication or physical measurement time. Evidence received_at is initial load
or update receipt; repeated publication retains identity. Source clock HOST_SYSTEM,
timestamp_kind ESTIMATE_UPDATE; session UUID is calibrator-process scope. Immutable
content SHA-256 is calibration_id. No samples => validity UNKNOWN, evidence invalid.
Historical fits remain historical; neither startup nor restoration invents fresh
physical observations.

## Integration API for parent

- `z70_manipulation.perception.front_camera(repo=None)`,
  `calibration(repo=None)`, `front_objects(image, repo=None, thresholds=None)`,
  `front_jaw(image, repo=None)` default to installed package resources. An explicit
  repo override is for archive/tests only. Remove executor `parents[4]` discovery;
  call without repo in production. Planner uses the same active gripper K/D.
- `z70_odometry.resources.runtime_calibration_path()` is the default mutable file:
  `$XDG_DATA_HOME/RRsZ70_ROS2/calibration/chassis/online.json`, or
  `~/.local/share/RRsZ70_ROS2/calibration/chassis/online.json` when unset.
  `output_file` remains the calibrator override. Do not launch with a checkout
  calibration path. Manipulation should subscribe to typed CalibrationState,
  not read repository online.json; gain consumers must honor known flags.
- Description owns the installed active front-camera JSON, camera-info YAML and
  unchanged floor-geometry module; odometry owns immutable historical chassis-fit
  seeds and mutable online state.

## Runtime API and migration

- `z70_description.resources.config_path(name)` resolves installed package share
  files. Missing installed data fails closed; source-mode fallback is confined to
  that package's config directory (no checkout ancestor walk). Pincher geometry
  now uses this same resolver, with no changes to geometry/model values.
- `FloorCamera.load()` and the odometry/mosaic `calibration` parameter default to
  the installed front-camera JSON. `initial_camera_x_m()` reads the retained
  **0.16 m initial VO guess** from `z70_odometry/config/odometry.yaml`. This is not
  silently replaced by the separate provisional 0.275 m model-preview mount.
  Existing estimation/ready gates and all geometry tolerances are unchanged.
- `seed_directory` defaults to installed historical chassis fits; set it to an
  empty string in synthetic integration tests to disable measured-history seeds.
- `initial_state_file` (default empty) explicitly imports an existing version-1
  online file **only when output_file does not exist**. Import is atomic,
  byte-preserving and no-clobber; there is no automatic checkout search. To retain
  existing observed samples on the new durable default, an operator can start
  the calibrator once with `-p initial_state_file:=/absolute/archive/online.json`.
  This run tested the import into scratch only, not the user's durable state.
- Saves fsync a private same-directory temporary, atomically replace and fsync
  the directory. Unchanged data is not rewritten. Restart/no-op publication
  retains update timestamps, content hashes and fit evidence; periodic state
  publication does not invent a new observation identity.
- `z70_odometry.calibration_state.gains_from_state(msg)` returns the existing
  executor gain-dictionary layout, honoring validity/stale/known flags. Parent
  manipulation node can use it in the typed calibration subscriber; if imported,
  declare its z70_odometry runtime dependency. `calibrated_camera_x(msg)` is now
  typed and used by both VO and mosaic. No String calibration publisher remains.

## Measured data and provenance

Original archive files remain untouched. Small installed runtime copies are
byte-identical; no images or full acquisition evidence corpus was copied.
Machine-readable manifests are installed with their owning packages:

- `z70_description/config/resources_provenance.json`: front JSON, camera-info,
  floor module, existing mount/pincher/lens configs with source paths and SHA-256.
- `z70_odometry/config/chassis/provenance.json`: the three historical fit JSONs.

| Original | SHA-256 |
|---|---|
| front-camera/front_camera.json | `8fa973327a3e412780d85de21a453e0f6efbdf3c6fea663f1035f09c379550b6` |
| front-camera/z70_front_camera_info.yaml | `958f3716c8e0baab506fe46e05306da7661fb8d4486bd7e7141ade8e3ff88b0f` |
| front-camera/floor_geometry.py | `13bf2ae48a0b4e77e6c5958df7127e4093578e60f60ad32eabb1193e936dd6a2` |
| chassis/20261013T144619Z/fit.json | `f9cc4acd12e53a7e1d1d99b11b6c1a128f12224669446fbc1c1bd71f522ff7b9` |
| chassis/20261013T151331Z/fit.json | `b260c6c78d4048046c5f58248e720a30b152a1795ed7385d4f1f300fbd859c46` |
| chassis/20261014T054025Z/fit.json | `726c03302970e059cfec34502778acaf6029f2981e8689d544d59786d995d202` |

Mutable legacy `calibration/chassis/online.json` was **not installed or edited**.
Its observed hash was
`3df23b7849c31560918ebbccb1c19274d14805a4ff29e246672d5376e01d03f2`.
The explicit scratch import retained all **15 samples / 490 seen command IDs**,
byte-identical initial content and original update time `1792043379.636437` after
seeding; source hash remained unchanged. This is a migration proof, not a live
calibration claim or an automatic copy of private runtime state.

Front CameraInfo's historical export rounds K to 3 decimal places and D to 6;
tests assert that exact export against the full-precision JSON. Gripper planner
K/D match the active adopted YAML exactly. No tolerance was changed.

## Executed offline verification

Using `/Users/mpentax/.pixi/bin/pixi` in `ros2/`:

- Source pure-Python perception/planner/operational geometry/resource and
  odometry/mosaic suites: **39 passed, 3 existing xfailed**. The historical 12px
  provisional-model expected failures remain unchanged.
- Built actual wheels for description/manipulation/odometry, laid out their
  purelib/share data in an isolated non-symlink scratch prefix, copied tests out
  of the checkout, set cwd to scratch and Z70_REPO to an absent path:
  **16 passed**; installed default front/floor projection and seed resolution
  succeeded. No source checkout was in the installed test process PYTHONPATH.
  Artifact prefix:
  `/Users/mpentax/.hermes/cache/scratch/phase3-resources-y2zzp5gk/installed`.
- Real archived online-state scratch import: **15/15 samples, 490/490 seen IDs**,
  unchanged original update time/source SHA; all checks passed.
- Scoped `git diff --check` passed. Original floor module and all existing
  mount/pincher/gripper config SHA-256s are unchanged.

Pixi lacks pip/installer, so wheel install used the standard purelib/data layout
via zip extraction (no network/dependency install). Initial test attempts exposed
missing ROS environment setup; applying environment after `pixi run` resolved it.
No colcon, robot/MCP, service, Docker, private state, live calibration or commits
were run by this resource worker. Wheel/unit proof is **not** a claim that the full
coordinated installed ROS suite passes.

## Required coordinated handoff (parent/interface owner)

Parent integration replaced the initial CalibrationState draft with the binding
schema above and compiled the generated types. Serialization and the typed DDS
calibrator integration pass in the coordinated installed suite (phase3.md).

Parent removed executor `parents[4]` calls (default installed perception APIs),
repository `online.json` reads, the hard-coded held-verification camera offset,
and bringup's checkout `output_file` override. Manipulation subscribes to the typed
gain adapter. Existing setup glob already
installs new description config files; odometry setup installs only its three
fit seeds, manifest and defaults. The resource worker did not edit parent-owned
executor/manipulation-node/bringup files; the parent made those call-site edits.
The full non-symlink suite subsequently passed; the phase-2 ten installed-resource
failures no longer occur. Historical optical expected failures remain unchanged.

