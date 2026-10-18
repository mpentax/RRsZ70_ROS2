# Stock Z70 RGB-only independent native DimOS mapping

Research: 2026-10-07. **No camera/robot RPC, live capture, movement, program, deployment, configuration change, model download or installation was performed.** Only local source inspection and public-source HTTP reads. This document supersedes the implication in `dimos-native-sock-pickup.md` that camera mapping first requires exposing depth or buying a sensor; it does not modify that separately owned document.

## Decision

**Stock RGB can support independent visual mapping. Native `DimSlam` explicitly supports monocular images today. However its exposed mono flow yields up-to-scale visual odometry, not a native dense floor occupancy map.** The first attainable native experiment is offline calibrated mono tracking plus native image memory; establish scale from independent measurements. Do not promise the existing voxel mapper can turn that odometry alone into walls/free-space.

Recommended primary: existing `DimSlam(camera_mode="mono", use_gpu=False)` replay of a passive, already-recorded, single rigid-camera sequence. This is traditional visual geometry/fusion, **not local neural-model inference**. Confirm whether the owner's no-local-inference preference permits conventional CPU SLAM; no neural fallback is implicitly authorized. If the mono path fails or a reusable sparse map/loop closure is required, the maintained **ORB-SLAM3 monocular CPU implementation** is the fallback, with a small typed integration, not new SLAM. It is external to DimOS, not falsely described as an already integrated native module.

Dense neural options DROID-SLAM and MASt3R-SLAM are real video SLAM, but are **not supported native DimOS modules in either inspected revision**, depend on local neural inference/GPU infrastructure, and do not fit the existing hosted-Copilot-only preference without a separately approved, actually available execution route. Depth Anything's native demo does not solve this gap.

The Roborock map is comparison-only: no occupancy input, initialization, scale calibration, pose prior, vendor trajectory supervision or feedback into reconstruction. Neither output is automatically ground truth.

## Evidence and revision boundary

Pinned checkout: `/Users/mpentax/.openclaw/workspace/dimos-v1/source`, `db3d0ca9f31d726bfea3a78e8a57662292372d50`. Separately inspected upstream archive: `/tmp/dimos-research-20261006/dimos-4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0`, the previously resolved latest-main revision. No claim this remains current at every later time, or that a running deployment was reverified. `dim_slam.py`, its Rust flake, and experimental `depth_estimator.py` compare byte-identical between these revisions. Source scan found no native DROID-SLAM/MASt3R integration; Android dependency strings are not DROID-SLAM evidence.

Immutable source index (all links are actual inspected files):

- [S1 DimSlam ports/config, pinned](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/dim_slam/dim_slam.py); [latest inspected counterpart](https://github.com/dimensionalOS/dimos/blob/4ee1d7f5eb3174400b7dcaef10bc759fd0d71da0/dimos/mapping/dim_slam/dim_slam.py).
- [S2 native handlers](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/dim_slam/rust/src/main.rs), [Cargo lock](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/dim_slam/rust/Cargo.lock).
- Cargo locks `dimensionalOS/dimSLAM` v0.4.1 to `7a8fc571fbd2b43fbdf9b99dc4f6eb5c014376d9`: [S3 visual core](https://github.com/dimensionalOS/dimSLAM/blob/7a8fc571fbd2b43fbdf9b99dc4f6eb5c014376d9/rust/src/cuvslam_odometry.rs), [composed fusion](https://github.com/dimensionalOS/dimSLAM/blob/7a8fc571fbd2b43fbdf9b99dc4f6eb5c014376d9/rust/src/dim_slam_core.rs).
- [S4 SDK platform flake](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/dim_slam/rust/flake.nix), [platform selector](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/utils/nvidia_env.py). Flake locks SDK repository `jeff-hykin/cu_vslam_rs` to `54b8c88762ae28e3d6165962b54503b28e328e7c`: [backend README](https://github.com/jeff-hykin/cu_vslam_rs/blob/54b8c88762ae28e3d6165962b54503b28e328e7c/README.md).
- [S5 camera-navigation guide](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/docs/capabilities/navigation/visual_nav_setup.md).
- [S6 experimental predicted-depth visualization](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/experimental/security_demo/depth_estimator.py).
- [S7 native memory](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/memory/module.py), [voxel mapper](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/voxels/module.py), [cost mapper](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/mapping/costmapper.py).
- [S8 native fiducial detector](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/perception/fiducial/marker_detection_stream_module.py), [marker pose](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/perception/fiducial/marker_pose.py), [TF publisher](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/perception/fiducial/marker_tf_module.py).

## Hardware versus exposed data

Official primary source, fetched 2026-10-07: [Roborock Saros Z70 product page](https://global.roborock.com/pages/roborock-saros-z70). Its HTML states: “dual cameras: one on the mechanical arm and one at the front of the vacuum,” with “camera switching”; OmniGrip has “precision sensors, a camera, and an LED light”; navigation is “StarSight Autonomous System 2.0”, “Next-Gen 3D Sensing”. Cameras are off by default and enabled by permission. The lightweight extractor returned title-only, so public HTML was inspected read-only. This is a mutable manufacturer page, not an immutable Git source; do not invent an immutable manufacturer citation.

These facts establish cameras and internal 3D sensing, **not API availability of metric depth, LiDAR, IMU, encoders or simultaneous stereo**. The inspected text does not establish exact RGB sensor specifications, ToF/structured-light protocol or calibration. Do not assert physically absent LiDAR or propose a firmware sensor switch. Internal vendor depth/sensing may exist while unavailable through this connector. Use the stock selectable color-image stream, with actual lens identity verified per recording. Front and moving-arm viewpoints are not a calibrated synchronized rigid stereo rig.

Local provenance: `docs/camera-observations.md`, `src/camera-observation.ts`, `src/go2rtc.ts`, `src/service.ts`. go2rtc JPEG bytes and service receipt/session/frame metadata are **image evidence**, not sensor exposure time, hardware sequence, verified lens, intrinsics or pose. A successfully decoded actual frame can become native `Image`; its existence alone cannot run SLAM. No actual frame was captured or invented here. A bounded project media search found no Z70 recording/JPEG fixture (only dependency artwork), so an existing recording must be supplied/located by its owner before replay. This is not proof none exists elsewhere.

## Actual native mono flow and its limits

`Image + CameraInfo + camera-to-rig/output TF → DimSlam mono/cuVSLAM → fused Odometry + TF → OdometryHist/native memory`. Optional `Imu`/external `Odometry` are not mandatory; use empty configs when unavailable. Do not synthesize encoder/IMU samples or integrate vendor command values into fake measured odometry.

S1 accepts `camera_mode="mono"`; S3 maps it to `CUV_ODOMETRY_MONO`, with one rig camera, and explicitly says “Mono is accurate only up to scale.” Color is handled as three-channel input, with conversion paths for tracker grayscale; images and camera info are selected by `frame_id`. A frame must be strictly newer than its predecessor. Calibration is real input, not inferred from image dimensions. Default `CameraConfig.rectified=True` **asserts** rectification: it does not rectify an arbitrary JPEG.

S2 publishes `depth_cloud` only in `handle_depth_image`. The wrapper does **not** publish a triangulated mono landmark cloud or a loop-closed reconstruction/map output. `map_frame_id` is carried configuration; S1 explicitly says no `map → odom` is published here. The internal tracker pose is fused via deltas. No automatic metric scale estimator is exposed. Thus a mono trajectory in an `Odometry` message whose nominal units say metres must still be marked unscaled until calibrated; ROS/LCM type names do not make scale physical.

A monocular trajectory and native semantic observation memory can map a room topologically (viewpoints, revisits, descriptions, image retrieval). Metric sparse reconstruction requires exposing an existing algorithm's genuine triangulated landmarks; dense floor occupancy requires additional genuine reconstructed surfaces and visibility/free-space accounting. Existing `VoxelGridMapper` expects posed world-frame clouds; odometry alone does not supply them. Sparse points alone should not be ray-carved into “all floor clear,” and unobserved walls remain unknown. Do not use floor-plane assumptions to invent walls or extrude the vendor raster.

The fusion core can consume real independently measured odometry sources, but is not documented as solving mono scale merely because a metric source is fused. Calibrate/validate scale explicitly; mixing unscaled visual deltas with metric poses is not a sound substitute. An IMU alone is not assumed to rescue scale here: tracker inertial mode is restricted to stereo/multisensor, while this composed module routes IMU to its filter.

## Algorithm / runtime compatibility matrix

| Existing implementation | Output / integration | CPU, Apple ARM64, GPU | Memory/compute and decision |
|---|---|---|---|
| Native DimSlam mono, cuVSLAM classical VO + fusion | Native image/calibration/TF → up-to-scale pose; no mono cloud output | Flake explicitly supports aarch64-darwin, aarch64-linux, x86_64-linux. Apple selector uses `metal`; locked fork supports CPU `use_gpu=False` and CuMetal GPU `True` from the same SDK | No neural weights. RAM scales with tracking state and recording; no measured Z70 throughput or minimum RAM established. **Primary offline flow**, conventional CPU SLAM permission caveat |
| Native DimSlam stereo/RGB-D/multisensor | Metric scale with calibrated baseline/actual depth; existing depth-cloud path | CPU stereo supported; locked backend README warns RGB-D needs GPU, CPU can report success with unchanged pose. Apple CuMetal is a fork path, not NVIDIA official macOS SDK | Not selected: stock selectable cameras are not proved stereo; no real depth stream |
| ORB-SLAM3 mono | Existing CPU sparse multi-map SLAM/loop closure; external adapter needed | Published Ubuntu build; C++ dependencies, no CUDA requirement. Native macOS ARM64 build not proved by cited source; Linux ARM64/container compatibility needs build/test, not assumed | Feature/vocabulary/map memory, not neural inference; no trustworthy fixed RAM budget from README. **Fallback** for sparse independent reconstruction; GPLv3 |
| DROID-SLAM | Real mono/stereo/RGB-D video SLAM; poses + reconstructed depths; external integration | Official installation requires CUDA/custom backend; not demonstrated CPU/MPS/macOS route | README: demo GPU ≥11 GB, some evaluations 24 GB. Local learned weights and GPU inference conflict with current preference. Not native/not free hosted Copilot functionality |
| MASt3R-SLAM | Real dense RGB MP4/image-folder SLAM, optional calibration, learned 3D priors/retrieval; external integration | Ubuntu/CUDA PyTorch/custom installs; experiments on RTX 4090. No official tested MPS/CPU substitute established | Large ViT checkpoint + retrieval assets + persistent keyframes/dense points. 4090 experiment is **not a proven minimum VRAM requirement**. Learned metric prior still needs independent scale checks; licenses/checkpoint restrictions apply |
| MASt3R alone | Pairwise learned geometry/matching and reconstruction tools, not interchangeable with sequential SLAM wrapper | CPU/no-CUDA demo route exists; that does not prove MASt3R-SLAM backend portability | Large local model, noncommercial CC BY-NC-SA code/checkpoint notices; not selected |
| Native security-demo DepthEstimator | Relative monocular depth `Image`, explicitly “Only intended for visualization (human consumption only)” | Chooses CUDA, MPS, CPU; HF transformer local model load | V2-Small, max inference width 640, latest-frame overwrite. Does not track poses/map, guarantee metric depth, temporal consistency or publish measured depth |
| Depth Anything V2 metric variants | Existing indoor/outdoor fine-tuned metric predictors; external integration, not the native relative demo | Local learned-model inference, MPS/CPU performance not measured here | Small 24.8M/Base 97.5M/Large 335.3M model family; parameter storage alone excludes activations/runtime/video buffers. Metric prediction ≠ measured range. Not a SLAM replacement |

External immutable primary references resolved/read during this research:

- [DROID-SLAM README](https://github.com/princeton-vl/DROID-SLAM/blob/2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99/README.md).
- [MASt3R-SLAM README](https://github.com/rmurai0610/MASt3R-SLAM/blob/e6f4e3d474fad0e11f561482012be864ba8c3f17/README.md).
- [MASt3R README](https://github.com/naver/mast3r/blob/f5209afc300cec36239a7ac992263f36847bbba0/README.md).
- [Depth Anything V2 README](https://github.com/DepthAnything/Depth-Anything-V2/blob/a561b849ebae10a6f5ef49e26c83cbbcd36c71bf/README.md), [metric variants](https://github.com/DepthAnything/Depth-Anything-V2/tree/a561b849ebae10a6f5ef49e26c83cbbcd36c71bf/metric_depth).
- [ORB-SLAM3 README](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/README.md).

Apple Metal support must not be incorrectly dismissed as universally “CUDA-only.” Conversely Apple Silicon Docker Linux does not automatically expose macOS Metal; aarch64 Linux CPU variant and native macOS CuMetal are different runtime choices. These are source-supported routes, not installed-build or performance proof. A backend return/status is insufficient: verify nonzero, correct-direction motion on fixture data.

## Scale and calibration without vendor-map leakage

Monocular geometry has a similarity-scale ambiguity. Choose independent anchors, record measurement uncertainty, and validate on held-out measurements:

1. Measured distances between reconstructed visible landmarks (e.g. two wall corners) or several survey positions. Fit scale/rigid alignment from surveyed anchors, not the Roborock raster. Use multiple distributed anchors to detect drift/nonuniform deformation.
2. Known camera height and independently measured floor plane can anchor a **reconstructed** plane/trajectory under a rigid mount. Height alone plus an arbitrary single image does not recover a whole room; floor visibility and a reliable plane estimate are required. Plane ray intersection yields floor points only for pixels truly on that plane, not arbitrary furniture depth.
3. Known-size fiducial with calibrated intrinsics gives metric camera-to-marker pose using native marker pose modules. A surveyed marker/world transform can anchor views; raw `MarkerTfModule` merely mirrors detections to TF and is not an automatic surveyed robot localizer. Composition/frame inversion/uncertainty still need validation. Use an already available printed/calibrated marker if any; no purchase is proposed.
4. A genuinely measured metric pose source, if later exposed, could help; commanded velocities or stale vendor map positions cannot stand in for it.

Per-lens calibration: intrinsics at actual decoded resolution/crop, lens distortion model, rectification, rigid camera-to-base transform and optical-axis convention. Record frame exposure/sequence or characterize container PTS/receipt offset/jitter; receipt time alone is not capture time. Preserve duplicates/drops/reordering and uncertainty. Check compression, rolling shutter, exposure changes, blur, texture, lighting, overlap and translation/parallax; pure rotation or stationary images cannot triangulate a floor map. Never combine front/arm images as stereo. A fixed arm pose may form a separate monocular camera session only if its rigidity/extrinsics are measured; pose changes split sessions or require genuine timestamped kinematics.

Predicted “virtual depth” can be a separately labelled learned hypothesis in a future approved experiment. It is not measured depth; do not pass a relative prediction into `depth_units_per_meter=1000` or advertise its point cloud as sensor geometry. Such conversion can create a plausible, systematically wrong occupancy map.

## Hosted vision and existing subscription

Native [OpenAIVlModel](https://github.com/dimensionalOS/dimos/blob/db3d0ca9f31d726bfea3a78e8a57662292372d50/dimos/models/vl/openai.py) sends encoded image content to a hosted OpenAI client. This proves image-upload implementation, not compatibility with GitHub Copilot subscription authentication/model routes. Do not buy credentials, repurpose tokens, upload private frames, or assume a paid API exists. A supported existing subscription image route needs a separately validated nonprivate fixture and user-authorized image disclosure.

Hosted VLM can label selected views/semantic rooms and socks, not substitute for SLAM's repeated feature matching, bundle adjustment, deterministic pose history or measured depth. DROID/MASt3R are not automatically accessible by sending an image to a chat model. Frame upload bandwidth + provider latency/queueing/rate limits + sampling lose temporal detail; keep hosted descriptions asynchronous to any geometric replay. Measure latency rather than claiming real-time operation. No hosted image request was made here.

## Concrete passive experiment and deliverables

**Stage 0, executable only once existing data is available:** obtain an already-recorded authorized single-camera clip/folder, its provenance and any existing calibration. No new session is started. Hash the original, preserve lens/session/frame IDs, dimensions and PTS; inspect decoding offline. If only one frame exists, report semantic-image feasibility and stop the geometric claim. If calibration is absent, report exactly what can be measured later; do not substitute guessed focal length as a validated calibration.

**Stage 1, native geometry:** on a permitted existing runtime, replay decoded native `Image`, real `CameraInfo` and measured static TF into existing DimSlam mono; leave IMU/odom source lists empty. Start with CPU, no learned depth. Configuration sketch (not deployed or run):

```python
DimSlam.blueprint(
    camera_mode="mono", use_gpu=False,
    cameras=[CameraConfig(frame_id="z70_front_optical", rectified=True)],
    rig_frame_id="z70_front_optical", output_frame_id="z70_front_optical",
    imus=[], odom_sources=[], publish_tf=False,
)
```

`rectified=True` is valid only for actually rectified fixture images. Using camera output frame intentionally avoids pretending an unmeasured base transform exists. This still requires valid calibration and a real compatible SDK build. No source sketch proves tracking. Record tracker lost/recovery counts, returned pose deltas, processing FPS/latency and peak RSS. Reject an unchanged pose on translating footage, teleports, wrong axes, mirrored motion and arbitrary false metres. Review the default covariance/speed gates against mono units; do not copy stereo metric thresholds blindly or disable them merely to hide failure.

**Stage 2, independent scale/semantic map:** retain raw unscaled trajectory, estimate scale from independent survey anchors, and preserve transform/residuals/uncertainty separately. Save native observation memory associated with verified pose provenance. Produce trajectory/keyframe visualization and semantic room/view graph. Call this the achieved artifact, not an occupancy grid.

**Stage 3, sparse/dense gate:** if the requirement remains floor occupancy, native mono's missing landmark/cloud output is an integration/product gap, not unavailable RGB hardware. Evaluate existing ORB-SLAM3 genuine sparse reconstruction before writing any SLAM; clearly distinguish triangulated structure from free-space. Dense neural reconstruction remains an optional future exception requiring approved compute/preferences/license route, not the primary stock-only deliverable. A small bridge may translate genuine existing outputs and uncertainty; no fake depth or point cloud.

**Stage 4, comparison only after construction:** freeze independent artifacts/hashes, then open the separate Roborock reference. Register for display/evaluation using independently measured common landmarks; report transform and anchor/held-out errors, observed floor/wall/doorway overlap, unknown coverage and changes in movable furniture. Do not use vendor map to choose scale while calling the result independent. If only sparse trajectory/semantic graph exists, compare appropriate qualitative landmarks/coverage; do not invent occupancy IoU.

No offline feasibility script was added: source directly proves the interface and a mocked config cannot prove geometry, calibration, SDK build or actual stock video. No suitable existing Z70 clip was located in the bounded search, so no physical-map result is claimed.

## Separately authorized future survey, not a present action

If no adequate old recording exists, request a distinct supervised stock-camera survey with a fixed lens and stable pose, slow translational overlap and loop revisits, suitable lighting/texture, exposure/timing provenance, measured independent anchors and pre/post calibration checks. Chassis/arm movement and camera session creation each remain separately authorized. No survey motion, switch, dock/stow or program is authorized by this research.

**What is missing now:** located passive sequential RGB artifact, per-lens calibration/rigidity, trustworthy timing, independent metric anchors, compatible runtime fixture proof, and a genuine reconstruction output if dense occupancy is required. **What is not established as a blocker:** absence of stock cameras, absence of native mono support, or a need to purchase LiDAR/depth hardware. An independent RGB trajectory/semantic room map is feasible in principle; a production metric occupancy map from current JPEG-only evidence is not yet demonstrated.
