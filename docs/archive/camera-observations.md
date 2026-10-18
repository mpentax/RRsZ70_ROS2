# Camera observation and calibration contracts

`src/camera-observation.ts` is dependency-free, pure evidence validation and geometry.
It contains no Z70 calibration, transport, or hardware access. Tests use an explicitly
synthetic downward-facing pinhole camera one meter above a synthetic floor.

## Current evidence gap

`Go2Rtc.frame()` returns bounded, JPEG-marker-checked bytes. It supplies no exposure
timestamp, sensor sequence, physical lens verification, dimensions, or calibration.
The service frame response includes bounded receipt metadata headers:
`X-Roboclaw-Frame-Id`, `X-Roboclaw-Camera-Session`, `X-Roboclaw-Received-At`,
`X-Roboclaw-Capture-Time: unknown`, `X-Roboclaw-Lens: unknown`, and encoded image
width/height when header parsing succeeds. MCP preserves opaque IDs and dimensions,
records adapter receipt time, and exposes `capturedAt: null`, `captureProvenance: null`,
and unknown lens verification. Those fields cannot establish fresh metric evidence.
Session IDs rotate on source reconfiguration, selected-device changes and explicit
stream/lens changes; they are local receipt scopes, not vendor producer identities.

## Receipt integration API

The owner of the actual frame-receipt path can use:

```ts
const bytes = await go2rtc.frame(); // Existing transport; this module never calls it.
const receivedAt = Date.now();     // Record immediately after complete byte receipt.
const observation = recordCameraFrameReceipt({
  identity: {
    source: { sourceId: deviceOpaqueId, lensId: null },
    streamId, sessionId, frameId,
  },
  dimensions: jpegImageDimensions(bytes),
  receivedAt,
});
```

Identifiers are non-secret opaque tokens, not credential-bearing URLs. The owner
must rotate `sessionId` on stream restart/reconfiguration and allocate a unique
`frameId` for each receipt. This local identity does **not** assert sensor sequence
or prove two JPEGs came from different exposures. `lensId: null` means unidentified;
a claimed lens token may be recorded but remains unverified. The receipt wrapper
always sets `lensVerification.status: "unknown"`, `capturedAt: null`, and
`captureProvenance: null`. `jpegImageDimensions` only parses supported 8-bit JPEG
SOF headers; it does not decode pixels or certify an image's contents.

All constructors defensively copy and deeply freeze their outputs. APIs revalidate
plain/JSON objects at use. They throw on invalid evidence; they do not silently
scale calibration, infer clocks, convert units, or attach a default lens.

## Metric admission

- `createCameraObservation`: verified lens evidence and exposure time require
  explicit provenance. Times are Unix milliseconds; exposure must precede receipt.
  Capture provenance is a synchronized camera clock or hardware trigger with a
  measured clock uncertainty bound, never receipt time.
- `createCameraCalibration`: binds physical source/lens and exact encoded resolution,
  pixel intrinsics, an explicit distortion model, measurement provenance and validity
  interval, and a measured angular ray-error bound. `none` asserts measured absence
  or rectification of distortion; it is not an unknown-distortion fallback.
- `assertMetricCameraObservation`: checks verified lens, source/resolution match,
  future timestamps, oldest possible exposure freshness, and calibration validity
  across the entire exposure-clock uncertainty interval. Policy is supplied explicitly.

The Brown–Conrady model uses `k1,k2,k3,p1,p2` in normalized coordinates and a measured
validity radius. A conservative Jacobian bound rejects configurations for which
unique inversion on that disk cannot be guaranteed. Out-of-domain, singular, or
nonconvergent inverse projections fail closed. This intentionally supports a small
well-conditioned subset of camera models rather than silently handling fisheye optics.

## Floor projection

`createFloorProjectionContext` binds all source/stream/session/frame fields to a
named capture-time base frame. `projectPixelToBase(observation, calibration,
context, pixel, policy)` returns immutable base-frame meters and a conservative
Euclidean uncertainty radius. Pixel centers span `[0,width-1] × [0,height-1]`;
fractional pixels are allowed within those bounds.

Camera axes are optical **x right, y down, z forward**. Base axes are **x forward,
y left, z up**. The caller provides a proper orthonormal camera-to-base rotation,
translation in **meters**, and a calibrated unit-normal plane in the same base
frame satisfying `normal · point + offset = 0` with meter offset. No map/world
transform, floor height, arm pose, or mounting position is assumed.

The context carries provenance for transform, plane, and uncertainty, all preceding
the earliest possible exposure. The caller must supply measurements applicable to
that exact frame, including a moving arm-mounted camera's pose, and certified
uncertainty bounds covering clock/motion effects. A copied frame identity is a
contractual assertion, not independent proof of that measurement. Calibration and
verification provenance tokens likewise reference externally established evidence;
numeric validation cannot authenticate their physical truth.

Uncertainty combines calibrated ray error, pixel-localization ray error and pose
rotation error (radians), camera position error (meters), plane-normal error
(radians), and plane offset error (meters). Rotation and normal errors become
unit-vector chord bounds; the quotient intersection propagates numerator and
denominator bounds. Projection rejects uncertain horizon crossings, origins whose
error spans the plane, intersections behind the camera, nonfinite values, excessive
range (including its error bound), and excessive output uncertainty. The projection
is conditional on the selected pixel actually lying on the calibrated floor: it
does not infer depth for obstacles or establish free space or robot localization.

Run the pure synthetic suite with `node --test src/camera-observation.test.ts`.
