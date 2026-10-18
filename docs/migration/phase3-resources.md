# Installed resources and durable calibration — owner reference

Current operations belong to [the ROS guide](../ros2.md),
[odometry](../../src/z70_odometry/README.md) and
[the arm-model guide](../z70-arm-model.md). This entrypoint preserves import
semantics and the original phase-3 receipts separately; historical hashes below
are not claims about today's active calibration or deployed robot.

## Resource and state ownership

- Description's [config resolver](../../src/z70_description/z70_description/resources.py)
  uses installed package resources, with package-local source fallback, not checkout
  ancestor discovery. Active intrinsics, mounts, CAD and contact proxies have
  distinct consumers; consult current arm-model documentation rather than the old
  migration assertion that geometry/model values never changed.
- Odometry's [resource/state implementation](../../src/z70_odometry/z70_odometry/resources.py)
  owns immutable historical chassis seeds and the mutable default
  `$XDG_DATA_HOME/RRsZ70_ROS2/calibration/chassis/online.json`, with
  `~/.local/share` fallback (relative XDG values are treated as unset).
  `output_file` explicitly overrides the destination; never point reproduction
  tests at a retained measurement archive or a user's runtime file.
- `initial_state_file` defaults empty. It imports a version-1 sample file only
  when `output_file` is absent: byte-preserving, atomic, no-clobber. It never
  searches the checkout. An existing destination is never overwritten by import.
  This is an explicit operator operation, not a silent migration or new observation.
- `seed_directory=''` disables historical seeds for isolated tests. Standalone
  `initial_camera_x_m()` reads the retained 0.16 m seed; ordinary bringup supplies
  the current mount value (0.275 m). A ready passive fit can replace initialization;
  neither source initialization alone is an independently surveyed physical offset.
- State saves use same-directory temporary data, fsync and atomic replacement;
  unchanged state is not rewritten. Retained update timestamps, sample identity
  and content hash survive restart/no-op publication.

## Calibration evidence contract

The exact schema is [CalibrationState.msg](../../src/z70_interfaces/msg/CalibrationState.msg),
not the old embedded draft. Its [publisher/adapter](../../src/z70_odometry/z70_odometry/calibration_state.py)
uses reliable transient-local publication and known/value flags for each gain,
uncertainty/count and camera offset. Consumers honor validity/staleness and known
flags. Calibration is ESTIMATED, not OBSERVED; fit uncertainty is not absolute
accuracy. No samples means unknown evidence, not zero error. Source time identifies
persisted estimate update, not periodic publication or physical exposure; a content
SHA-256 identifies calibration content. Manipulation consumes typed calibration,
not checkout `online.json`.

## Historical phase-3 resource receipts (reported, not rerun)

Everything below records that migration's originals, hashes, scratch import and
software checks unchanged. The scratch prefix
`/Users/mpentax/.hermes/cache/scratch/phase3-resources-y2zzp5gk/installed` is an
unavailable/ephemeral historical output, not a portable install prefix. Old `ros2/`
commands and coordinated-worker instructions are not present-day instructions.
No historical hash, sample count or optical expected failure is upgraded here.

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

## Historical executed offline verification

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

## Historical coordinated handoff (completed migration)

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

