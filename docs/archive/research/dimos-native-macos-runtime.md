# Native macOS DimOS CPU/Metal assessment

Research date: 2026-10-07. Read-only hardware/environment/source checks and public locked-source HTTP reads; only this research document was written. No installation, build, restart, deployment, camera request, recording, robot RPC, motion, credential change or model download. No benchmark was run.

## Recommendation

**Keep the current Docker robot service/controller/relay and hosted Copilot Luna workers. Add a separately qualified native macOS DimSlam replay process first, rather than migrate all DimOS.** The Mac has suitable Apple Silicon/Metal hardware and the pinned backend has a real downloadable CuMetal SDK, not merely an aspirational source port. However no working native SLAM installation or M5/macOS 27 execution is proved on this host. “Can use Metal” means a concrete installation/qualification route exists; it does not mean it is enabled now or faster.

SLAM here is conventional geometric tracking/fusion, not local LLM inference. The hosted Luna model route and host-held subscription authentication remain unchanged. Moving geometry to the host does not authorize learned depth/vision checkpoints, local LLMs or camera uploads. Linux Docker on Apple Silicon does not provide macOS Metal access. Its ARM64 CPU backend and native macOS CuMetal backend are distinct.

## Live hardware and readiness evidence

`system_profiler SPHardwareDataType SPDisplaysDataType` and `sw_vers` reported:

- Mac mini, model identifier `Mac17,16`, Apple M5 Pro.
- CPU: 15 cores (5 Super and 10 Performance); GPU: 16 cores.
- Unified memory: 24 GB; GPU advertises Metal 4.
- macOS 27.0.1, build 26A434; runtime architecture arm64.

Do not replace these live values with assumptions about an older M-series machine. Unified memory is shared among OS, Docker, geometry and graphics, not 24 GB of dedicated free VRAM. No working-set budget or throughput was measured.

Existing `workspace/dimos-v1/z70/.venv/bin/python` is native CPython 3.12.15, arm64/darwin. Metadata/module probes found no installed `dimos`, `roboclaw-dimos`, NumPy, OpenCV contrib, dimos-lcm, Zenoh, aioquic, Open3D, PyTurboJPEG, pin or gtsam-extended in that environment. Module specs for cv2/lcm/zenoh/aioquic/dimos were absent. Thus there is no native import success to report. The separate inference venv is Python 3.9.6; do not reuse it as a DimOS Python 3.12 environment or alter a working inference worker.

`nix`, `uv`, `deno`, `python3.12` were not resolved on the execution shell PATH; checked usual `/opt/homebrew/bin` tool paths and default Nix profile did not supply them. This is a bounded discovery result, not proof no executable exists elsewhere. A native Python 3.12 interpreter already exists inside the z70 venv. No SLAM/Metal artifacts were found by the bounded Nix-store check. No GPU runtime smoke test was performed.

## Pin and actual backend selection

Inspected upstream checkout: `/Users/mpentax/.openclaw/workspace/dimos-v1/source`, exact HEAD `db3d0ca9f31d726bfea3a78e8a57662292372d50`. Owned deployment package: `/Users/mpentax/Code/RoboRockZ70-Connector/integrations/dimos`; its README targets the same pin. Keep these revisions separate from latest main.

Pinned `DimSlamConfig` defaults to `result/bin/dim_slam`, cwd `rust`, and a Nix build selected by `sdk_variant()`. On darwin/arm64 the selector explicitly returns `metal`; Intel macOS explicitly raises unsupported. Darwin driver environment sets `CUMETAL_USE_METAL_DEVICE_ADDRESSES=1`. Its flake lists `aarch64-darwin`, `aarch64-linux`, `x86_64-linux`, and links the native Rust module to the selected SDK through `CUVSLAM_SDK_DIR` and a dylib/library rpath.

There are **two separate choices**:

1. SDK platform selection is automatic (`metal` on Apple Silicon).
2. Tracker execution is selected explicitly by `use_gpu`: default true, set false for CPU baseline. The locked Metal SDK has `ENFORCE_GPU=OFF`, carrying CPU and CuMetal paths in the same library. No generic PyTorch MPS flag selects this tracker. A mismatched one-backend library can yield a warning/fallback; validate which backend actually ran.

The DimSlam Cargo lock pins dimSLAM 0.4.1 to `7a8fc571fbd2b43fbdf9b99dc4f6eb5c014376d9`. The SDK flake locks `jeff-hykin/cu_vslam_rs` to `54b8c88762ae28e3d6165962b54503b28e328e7c`. That locked flake uses the release:

`https://github.com/jeff-hykin/cu_vslam_rs/releases/download/cuvslam-v17.0.0-metal.5/cuvslam-cpp-17.0.0-arm64-metal-macos.tar.gz`

with hash `sha256-ugma/HyDi7caSXwVCMCtZB3MAi93BWbMcbvSrLYL1Xw=`. Read-only HEAD followed redirects to HTTP 200 and content length 6,567,232 bytes. This proves artifact availability at inspection time, not downloaded-byte hash verification or successful execution. The artifact targets macOS 13/apple-m1 per flake comments; M5 Pro/macOS 27 are not individually qualified by that statement.

This is a **community cuVSLAM v17 fork/CuMetal route**, not NVIDIA's official macOS SDK. The archive supplies `libcuvslam.dylib`, `libcumetal.dylib`, headers and prewarmed `share/cumetal-cache`; the Nix package preserves these and license notices. CPU does not imply the Metal SDK can be stripped of its linked runtime libraries. Caches must match the exact libcumetal build. A full source rebuild/prewarming needs Xcode's Metal compiler; the prewarmed release exists precisely to avoid that inside the Nix sandbox. The module still needs its own native Rust build and dependency closure; the 6.6 MB SDK is not the size of the whole installation.

CuMetal creates a writable registration/JIT cache on first use, even when prebuilt kernels suffice. For future qualification isolate `CUMETAL_CACHE_DIR`, keep the bundled cache, and test without `DEVELOPER_DIR` to distinguish genuine prebuilt-cache loading from local compilation. Upstream's `metal_smoke` is GPU-only stereo and checks actual motion rather than success status; its reported test results are upstream evidence, not a benchmark on this Mac. RGB-D CPU can falsely return success with unchanged pose: do not use it as a CPU comparison. The Z70 task is monocular, not proved RGB-D/stereo.

## Python/native dependency portability

DimOS 0.0.14 declares Python >=3.10,<3.13; the owned package declares >=3.12. **Python 3.12 arm64 is their valid intersection.** Native wheels/dylibs must be macOS arm64/universal2, not Linux aarch64 or Rosetta x86_64. The lock contains platform-specific alternatives; wheel existence is not an import/build guarantee or a reason to blindly install every extra.

Actual `uv.lock` inventory:

| Component | Locked macOS evidence / constraint |
| --- | --- |
| dimos-lcm 0.1.4 | `py3-none-any` wheel; generated message layer is platform-independent |
| lcm-dimos-fork 1.5.2.post1 | cp312 macOS 14 arm64 wheel; native dependency underneath dimos-lcm |
| eclipse-zenoh 1.10.1 | cp39 abi3 universal2/arm64 wheel compatible in principle with cp312 |
| OpenCV contrib 4.13.0.92 | cp37 abi3 macOS 13 arm64 wheel |
| Open3D 0.19.0 | cp312 macOS universal2 wheel; Linux ARM uses separately marked unofficial package |
| pin 4.1.0 | cp312 macOS 11 arm64 wheel; accompanying cmeel C++ libraries/ABI still matter |
| gtsam-extended 4.3a1.post1 | cp312 arm64 and universal2 options in lock; mapping extra, not mandatory for every SLAM wrapper |
| imagecodecs | macOS arm64 cp312/abi3 options across lock branches; select correct Python/platform marker branch |
| llvmlite 0.46.0 | cp312 macOS 11 arm64 wheel, supporting occupancy dependencies |
| rerun-sdk 0.32.0 / dimos-viewer 0.32.0a2 | macOS arm64 wheels; currently core dependencies despite visualization-only intent |
| aioquic 1.3.0 | cp310 abi3 macOS 11 arm64 wheel, web extra for stock relay client |
| PyTurboJPEG 1.8.2 | sdist in lock, Python wrapper additionally needs a discoverable compatible libturbojpeg dylib; wheel inventory alone is insufficient |

Core includes OpenCV, Open3D, pin/pin-pink, scipy, numba/llvmlite and viewer packages even for a small application. Test the actual minimal native DimSlam imports first; do not assume no extras means a lightweight dependency footprint. CPU geometry does not require the `cpu` extra's ONNX inference backend.

Do **not** start from `[all]`, developer dependency groups, or copy the Linux image's site-packages. Optional perception/misc/agents stacks can bring torch, transformers, segmentation models, audio and local learned inference. CUDA extras (`cupy-cuda12x`, `onnxruntime-gpu`) do not confer Metal portability. Their current markers are x86_64-only, not a universal Linux/macOS portability promise. `a750-control` is explicitly Linux x86_64; Jetson wheel extras are commented out; DDS/unitree-dds require special setup. Linux CAN/ROS/hardware interfaces need their own platform qualification. macOS PyTorch MPS support in some models is not general CUDA/custom-kernel support, and does not prove DROID/MASt3R can run here.

ROS is optional (`ROSTransport` is available separately); ROS-like message names do not require migrating the robot service into ROS. Stock LCM/Zenoh typed transports are already present in Python and native Rust modules. Deno 2.6.10 belongs to the stock web/relay build/run path; it is not required solely to execute an independently built geometric SLAM process. Keep the working relay rather than add a second relay or auto-download Deno through `--local-relay` during a supposedly read-only probe.

## Hybrid integration boundary

Retain existing Linux/arm64 `roboclaw-dimos:arm64` controller and TLS-authenticated upstream relay, existing host MCP bridge and host hosted-Luna inference bridge. Deployment compose has 2 CPU/2 GB container limits and loopback TCP+UDP 7780 with individual relay artifacts and queue/status mounts. These are configuration observations, not this document's live service health claim. Whole migration could remove container isolation, change file paths/process lifecycle, expose discovery/listeners and disturb correlation/receipts without making vision inputs better.

A native SLAM worker should consume validated typed Image, CameraInfo and static TF and publish typed Odometry/TF, using existing native DimOS transports. Prefer explicit **Zenoh TCP endpoints** for a Docker-to-host process boundary: both native Rust and Python support it; configure discovery/listeners deliberately, loopback-only published ports and matching schema/topic/QoS. Its default high-rate Image latest-wins behavior is suitable for live bounded streams but can invalidate an all-frames replay benchmark; pace replay and report drops. LCM is also supported, but Docker Desktop UDP multicast traversal is not assumed; test explicit networking before choosing it. Shared-memory transport is only for compatible processes sharing the same OS/memory namespace, not a Docker-VM-to-host substitute.

The stock WebTransport `RelayClient`/aioquic handles cockpit/controller streams and existing TLS auth; do not write another QUIC stack or assume its stream envelope automatically becomes a DimSlam CameraInfo/TF feed. The currently owned Z70 state/JPEG stream still needs validated conversion/calibration/provenance. A small typed wiring adapter is appropriate, not a new mapping engine or bespoke transport protocol. No new network endpoint is currently authorized/deployed by this document.

Keep RoboClaw service token references, Copilot auth and model configuration on the host in their existing scope. Never copy them into a native SLAM environment, relay stream, wheel or container. Relay robot/viewer keys are separate; existing broad viewer authority is not a read-only geometry scope. Do not relax TLS, add tokens, alter vendor access or forward actuation topics just to move compute.

## Staged proof, with stop gates

1. **Dependency-only isolated qualification:** create a dedicated project-local Python 3.12 arm64 environment after authorizing installation/build work; retain pin and lock. Resolve only the required DimSlam core/runtime dependency set, verify actual native imports, extension architectures and dylib linkage (`file`, `otool -L`), including libturbojpeg, LCM and Zenoh. Use the pinned Nix recipe if Nix is approved/available; installing Nix is a separate host change, not silently done here. A manual SDK/Rust build is an alternative only after honoring exact lock/SDK/runtime/license layout, not evidence it already works. No system-wide pip/Homebrew changes are needed merely for this assessment.
2. **No-camera backend fixture:** qualify CPU and GPU against an existing synthetic/nonprivate fixture, checking nonzero correctly directed poses and backend identity. Use upstream GPU-only smoke tests where applicable; do not accept silent CPU fallback or status-only success. Record M5/macOS version, package pin, SDK hash, architecture, caches and runtime linkage.
3. **Passive mono replay:** use an already recorded authorized single rigid lens clip plus real intrinsics/distortion/rectification/timing/extrinsics. The prior bounded research found no suitable existing Z70 clip; obtain/locate one before promising this comparison. No new recording or motion is authorized here. Replay identical decoded frames/config CPU `use_gpu=False` versus CuMetal `use_gpu=True`; preserve raw up-to-scale poses. Record warm-up separately, frame counts/drops, processing FPS, median/p95 latency, peak RSS/unified-memory pressure, CPU load, tracker losses/recovery, motion/trajectory consistency and independently anchored accuracy. Fix playback pacing and include transfer/decoding overhead. Report cold versus warm cache separately and repeat; faster runtime with worse tracking is not a win.
4. **Only then live architectural decision:** if Metal materially improves this actual workload without unacceptable memory/accuracy/regression, attach the native geometric worker over stock transport to existing service/relay. If CPU meets needs, prefer simpler CPU. If native dependency/runtime qualification fails, retain Docker CPU and report the exact import/build/library failure. Do not migrate hosted chat/controller/robot control merely to access the GPU.

Illustrative native build command, **not executed**, once Nix and dependencies are explicitly approved/available:

```sh
cd /Users/mpentax/.openclaw/workspace/dimos-v1/source/dimos/mapping/dim_slam/rust
nix build -L 'git+file:../../../..?dir=dimos/mapping/dim_slam/rust#metal'
```

The module's configurable executable/build-command seam can point at a separately qualified binary without patching upstream/vendor code. Build sees tracked files only. An isolated venv alone cannot isolate a host Nix installation or repair missing system libraries; explain and authorize those separately if actually required.

## What Metal cannot fix

As established in `z70-rgb-only-native-mapping.md`, mono DimSlam yields up-to-scale odometry and TF, **not an exposed mono landmark cloud, loop-closed map-to-odom or dense floor occupancy map**. Moving it to Metal does not add those outputs. It does not manufacture metric scale, lens calibration, capture timestamps, robot extrinsics, IMU/encoder data, stereo synchronization or depth. Front and moving-arm cameras are not a proved rigid stereo pair; changing arm pose breaks a fixed mono extrinsic model. A faster trajectory still needs independently surveyed scale and accuracy validation.

Missing sparse/dense reconstruction export is an algorithm/interface product gap, not a CPU-vs-GPU issue. Preserve the staged reconstruction and ORB-SLAM3 fallback assessment rather than rewriting SLAM or injecting guessed/learned depth as measured geometry. Geometry and hosted semantic vision remain separate; neither Metal nor Luna inference proves physical completion.

## Source references

Immutable upstream pin paths: [DimSlam config](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/dim_slam/dim_slam.py), [hardware selector](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/utils/nvidia_env.py), [native flake](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/dim_slam/rust/flake.nix), [Python dependency declarations](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/pyproject.toml), [wheel lock](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/uv.lock), [transport factory](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/core/transport_factory.py), [Rust transport selection](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/native/rust/dimos-module/src/lib.rs).

Locked SDK [README](https://github.com/jeff-hykin/cu_vslam_rs/blob/54b8c88762ae28e3d6165962b54503b28e328e7c/README.md) and [flake/artifact declaration](https://github.com/jeff-hykin/cu_vslam_rs/blob/54b8c88762ae28e3d6165962b54503b28e328e7c/flake.nix). Local owned `integrations/dimos/{README.md,pyproject.toml}` and `workspace/dimos-v1/deployment/compose.yaml` were inspected without secret file reads. Upstream SDK license is NVIDIA Community License, crate Apache-2.0; preserve notices rather than assume all binaries inherit the owned package's Apache license.

## Implementation follow-up — offline qualification, 2026-10-07

The isolated implementation now lives in `integrations/dimos/native_slam/`. Its doctor checks pinned bytes, architecture and linkage without loading SDK code; strict qualify/replay fail closed. Dataset replay planning preserves actual monotonic receipt intervals, unknown exposure, lens calibration and up-to-scale semantics. No camera, robot status/RPC, motion, mode or service changes were made.

**Correction to the recommendation above:** artifact availability and a Metal community port do not establish legal permission for Apple execution. The downloaded release archive hash matches the pinned flake (`ba099afc7c838bb71a497c1508c0ad641dcc022f770566cc71bbd2acb60bd57c`), but its packaged NVIDIA Community License §2 explicitly grants use solely on NVIDIA GPU/CPU platforms or NVIDIA HCA networking systems. API headers independently carry the NVIDIA-hardware-only condition. This Apple M5 host is not shown to satisfy that condition. Stop SDK loading/build/execution pending applicable permission/licensing (or a different adequately licensed backend). No actual Metal benchmark or GPU initialization was run; no CPU fallback was substituted.

Private `workspace/dimos-v1/native-slam/.venv` is CPython 3.12.15 arm64 and stdlib-only. Both SDK dylibs are arm64, with CuMetal linking system Metal/MPS frameworks; 65 packaged metallib cache files and license retained privately. Native DimOS dependency imports are not installed in this new venv. Cargo was unavailable on execution PATH/checked customary locations, and `xcrun --find metal` failed; no global toolchain installation occurred. Nine owned offline gate/timing/contract tests pass, and the combined sibling capture/dataset suite passes all 18 tests. See native_slam README for the staged upstream synthetic stereo test, explicit backend evidence requirements and remaining typed replay/build/dependency gates. This is scaffolding plus a concrete stop gate, not an enabled SLAM worker or qualified M5 backend.
