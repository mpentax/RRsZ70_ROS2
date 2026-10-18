import { test } from "node:test";
import assert from "node:assert/strict";
import {
  assertMetricCameraObservation, createCameraCalibration, createCameraObservation,
  createFloorProjectionContext, jpegImageDimensions, projectPixelToBase, recordCameraFrameReceipt,
} from "./camera-observation.ts";
import type {
  CameraCalibration, CameraFrameIdentity, CameraObservation, FloorProjectionContext, FloorProjectionPolicy,
} from "./camera-observation.ts";

const identity: CameraFrameIdentity = {
  source: { sourceId: "synthetic-device", lensId: "synthetic-lens" },
  streamId: "synthetic-stream", sessionId: "session-1", frameId: "frame-1",
};
const provenance = { evidenceId: "synthetic-measurement", measuredAt: 900 };
const calibration: CameraCalibration = {
  calibrationId: "synthetic-calibration", source: identity.source, dimensions: { width: 201, height: 101 },
  intrinsics: { fx: 100, fy: 100, cx: 100, cy: 50, units: "pixels" }, distortion: { model: "none" },
  provenance, validFrom: 900, validUntil: 2000, rayAngularErrorRad: 0,
};
const observation: CameraObservation = {
  identity, dimensions: calibration.dimensions, receivedAt: 1010, capturedAt: 1000,
  captureProvenance: { method: "hardware-trigger", evidenceId: "synthetic-trigger", clockUncertaintyMs: 1 },
  lensVerification: { status: "verified", provenance },
};
const context: FloorProjectionContext = {
  identity, baseFrameId: "base-at-frame-1",
  // Synthetic downward optical axis. Camera right = base right; camera down = base backward.
  cameraToBase: { rotation: [[0, -1, 0], [-1, 0, 0], [0, 0, -1]],
    translation: [0, 0, 1], translationUnits: "meters", provenance },
  floorPlane: { normal: [0, 0, 1], offset: 0, units: "meters", provenance },
  uncertainty: { pixelRayAngularErrorRad: 0, poseAngularErrorRad: 0, positionErrorMeters: 0,
    planeNormalErrorRad: 0, planeOffsetErrorMeters: 0, provenance },
};
const policy: FloorProjectionPolicy = {
  now: 1020, maxCaptureAgeMs: 100, maxRangeMeters: 5, maxUncertaintyMeters: 0.1, minIncidenceCosine: 0.1,
};
const center = { x: 100, y: 50 };
function project(pixel = center, o = observation, c = calibration, f = context, p = policy) {
  return projectPixelToBase(o, c, f, pixel, p);
}
function close(actual: number, expected: number, tolerance = 1e-10) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);
}

test("byte receipt remains unverified and cannot substitute receipt recency for exposure", () => {
  const receipt = recordCameraFrameReceipt({ identity, dimensions: calibration.dimensions, receivedAt: 1020 });
  assert.equal(receipt.capturedAt, null);
  assert.equal(receipt.captureProvenance, null);
  assert.deepEqual(receipt.lensVerification, { status: "unknown" });
  assert.throws(() => assertMetricCameraObservation(receipt, calibration, policy), /capture time is unknown/);
  assert.throws(() => project(center, { ...observation, lensVerification: { status: "unknown" } }), /lens is unverified/);
});

test("contracts defensively copy and deeply freeze source, frame, calibration, and projection evidence", () => {
  const mutable = structuredClone(observation);
  const o = createCameraObservation(mutable);
  (mutable.identity.source as { lensId: string }).lensId = "changed";
  assert.equal(o.identity.source.lensId, "synthetic-lens");
  const c = createCameraCalibration(calibration), f = createFloorProjectionContext(context), result = project();
  for (const value of [o, o.identity, o.identity.source, o.dimensions, o.captureProvenance, o.lensVerification,
    c, c.intrinsics, c.distortion, c.provenance, f, f.cameraToBase, f.cameraToBase.rotation,
    f.cameraToBase.rotation[0], f.cameraToBase.translation, f.floorPlane, f.uncertainty,
    result, result.identity.source, result.pixel, result.point]) assert.ok(Object.isFrozen(value));
  assert.throws(() => { (o.identity.source as { lensId: string }).lensId = "wrong"; }, TypeError);
  const mutableContext = structuredClone(context);
  const copy = createFloorProjectionContext(mutableContext);
  (mutableContext.cameraToBase.translation as unknown as number[])[2] = 99;
  assert.equal(copy.cameraToBase.translation[2], 1);
});

test("invalid dimensions, identities, clock provenance, and lens claims reject at admission", () => {
  for (const width of [0, -1, 1.2, NaN, Infinity, Number.MAX_SAFE_INTEGER + 1]) {
    assert.throws(() => createCameraObservation({ ...observation, dimensions: { width, height: 101 } }), /dimensions/);
  }
  for (const badId of ["", " ", "http://camera.invalid", "secret?pin=1234"]) {
    assert.throws(() => createCameraObservation({ ...observation, identity: { ...identity, streamId: badId } }), /identity/);
  }
  assert.throws(() => createCameraObservation({ ...observation, capturedAt: 1011 }), /later than receipt/);
  assert.throws(() => createCameraObservation({ ...observation, capturedAt: null }), /capture provenance/);
  assert.throws(() => createCameraObservation({ ...observation, captureProvenance: null }), /capture provenance/);
  assert.throws(() => createCameraObservation({ ...observation, receivedAt: Infinity }), /finite/);
  assert.throws(() => createCameraObservation({ ...observation, capturedAt: -1 }), /nonnegative/);
  assert.throws(() => createCameraObservation({ ...observation, capturedAt: 1000.1 }), /Unix milliseconds/);
  assert.throws(() => createCameraObservation({ ...observation,
    identity: { ...identity, source: { ...identity.source, lensId: null } } }), /lens verification/);
  assert.throws(() => createCameraObservation({ ...observation,
    lensVerification: { status: "verified", provenance: { ...provenance, measuredAt: 1001 } } }), /lens verification/);
  assert.throws(() => createCameraObservation({ ...observation, captureProvenance: {
    ...observation.captureProvenance!, clockUncertaintyMs: -1 } }), /clock uncertainty/);
  assert.throws(() => createCameraObservation({ ...observation, captureProvenance: {
    ...observation.captureProvenance!, method: "receipt" as "hardware-trigger" } }), /clock provenance/);
});

test("freshness uses the oldest possible exposure and calibration covers the full clock interval", () => {
  assert.doesNotThrow(() => assertMetricCameraObservation(observation, calibration, { now: 1099, maxCaptureAgeMs: 100 }));
  assert.throws(() => project(center, observation, calibration, context, { ...policy, now: 1100 }), /stale capture/);
  assert.throws(() => project(center, { ...observation, receivedAt: 1021 }), /future observation/);
  assert.throws(() => project(center, observation, calibration, context, { ...policy, maxCaptureAgeMs: NaN }), /finite/);
  assert.throws(() => project(center, observation, { ...calibration, validFrom: 1000 }), /calibration validity/);
  assert.throws(() => project(center, observation, { ...calibration, validUntil: 1000 }), /calibration validity/);
  assert.throws(() => project(center, { ...observation,
    lensVerification: { status: "verified", provenance: { ...provenance, measuredAt: 1000 } } }), /earliest capture/);
  assert.throws(() => project(center, { ...observation, captureProvenance: {
    ...observation.captureProvenance!, clockUncertaintyMs: 21 } }), /future observation/);
});

test("calibration requires physical lens, measured provenance, resolution, units, finite intrinsics and distortion", () => {
  for (const fx of [0, -1, Infinity, NaN]) {
    assert.throws(() => createCameraCalibration({ ...calibration, intrinsics: { ...calibration.intrinsics, fx } }), /fx/);
  }
  assert.throws(() => createCameraCalibration({ ...calibration, source: { ...identity.source, lensId: null } }), /physical lens/);
  assert.throws(() => createCameraCalibration({ ...calibration, intrinsics: { ...calibration.intrinsics, cx: 201 } }), /principal point/);
  assert.throws(() => createCameraCalibration({ ...calibration, intrinsics: {
    ...calibration.intrinsics, units: "meters" as "pixels" } }), /units/);
  assert.throws(() => createCameraCalibration({ ...calibration, validUntil: 900 }), /validity interval/);
  assert.throws(() => createCameraCalibration({ ...calibration, provenance: { ...provenance, measuredAt: 901 } }), /provenance/);
  assert.throws(() => createCameraCalibration({ ...calibration, rayAngularErrorRad: NaN }), /finite/);
  const distortion = { model: "brown-conrady" as const, k1: 0.05, k2: 0, k3: 0, p1: 0, p2: 0, maxRadiusNormalized: 1 };
  assert.throws(() => createCameraCalibration({ ...calibration, distortion: { ...distortion, p1: NaN } }), /finite/);
  assert.throws(() => createCameraCalibration({ ...calibration, distortion: { ...distortion, k1: -1 } }), /unique inversion/);
  assert.throws(() => createCameraCalibration({ ...calibration, distortion: { ...distortion, maxRadiusNormalized: 0 } }), /positive/);
  assert.throws(() => project(center, observation, { ...calibration, dimensions: { width: 202, height: 101 } }), /resolution mismatch/);
  for (const source of [{ ...identity.source, sourceId: "other-device" }, { ...identity.source, lensId: "other-lens" }]) {
    assert.throws(() => project(center, observation, { ...calibration, source }), /source\/lens mismatch/);
  }
});

test("known synthetic pinhole projects center and boundary pixels into base meters", () => {
  const result = project();
  assert.deepEqual(result.point, [0, 0, 0]);
  assert.equal(result.rangeMeters, 1);
  assert.equal(result.uncertaintyRadiusMeters, 0);
  assert.equal(result.units, "meters");
  assert.equal(result.capturedAt, 1000);
  assert.equal(result.baseFrameId, context.baseFrameId);
  assert.deepEqual(result.identity, identity);
  for (const [pixel, expected] of [
    [{ x: 0, y: 0 }, [0.5, 1, 0]], [{ x: 200, y: 100 }, [-0.5, -1, 0]],
    [{ x: 150, y: 50 }, [0, -0.5, 0]],
  ] as const) {
    const point = project(pixel).point;
    point.forEach((v, index) => close(v, expected[index]));
  }
  const raised = project(center, observation, calibration, { ...context,
    floorPlane: { ...context.floorPlane, offset: -0.2 } });
  close(raised.point[2], 0.2); close(raised.rangeMeters, 0.8);
});

test("Brown-Conrady inverse recovers a known synthetic distorted pixel", () => {
  const d = { model: "brown-conrady" as const, k1: 0.05, k2: -0.005, k3: 0.001,
    p1: 0.002, p2: -0.003, maxRadiusNormalized: 1 };
  const x = 0.4, y = 0.2, r2 = x * x + y * y;
  const radial = 1 + d.k1 * r2 + d.k2 * r2 ** 2 + d.k3 * r2 ** 3;
  const xd = x * radial + 2 * d.p1 * x * y + d.p2 * (r2 + 2 * x * x);
  const yd = y * radial + d.p1 * (r2 + 2 * y * y) + 2 * d.p2 * x * y;
  const result = project({ x: 100 + xd * 100, y: 50 + yd * 100 }, observation, { ...calibration, distortion: d });
  close(result.point[0], -y); close(result.point[1], -x); close(result.point[2], 0);
  assert.throws(() => project({ x: 200, y: 100 }, observation,
    { ...calibration, distortion: { ...d, maxRadiusNormalized: 0.5 } }), /distortion calibration domain/);
  const negativeRadial = { ...d, k1: -0.05, k2: 0, k3: 0, p1: 0, p2: 0 };
  close(project({ x: 139.6, y: 69.8 }, observation, { ...calibration, distortion: negativeRadial }).point[1], -0.4);
});

test("out of bounds, negative, and nonfinite pixels reject", () => {
  for (const pixel of [{ x: -0.01, y: 50 }, { x: 201, y: 50 }, { x: 100, y: -1 },
    { x: 100, y: 101 }, { x: NaN, y: 50 }, { x: 100, y: Infinity }]) {
    assert.throws(() => project(pixel), /outside image|finite/);
  }
});

test("projection binds source, lens, stream, session and exact frame; provenance must precede exposure", () => {
  for (const changed of [
    { ...identity, frameId: "frame-2" }, { ...identity, sessionId: "session-2" },
    { ...identity, streamId: "stream-2" }, { ...identity, source: { ...identity.source, lensId: "other" } },
    { ...identity, source: { ...identity.source, sourceId: "other" } },
  ]) assert.throws(() => project(center, observation, calibration, { ...context, identity: changed }), /frame mismatch/);
  for (const key of ["cameraToBase", "floorPlane", "uncertainty"] as const) {
    assert.throws(() => project(center, observation, calibration, { ...context,
      [key]: { ...context[key], provenance: { ...provenance, measuredAt: 1000 } } }), /postdates capture/);
  }
});

test("transform rejects wrong units, reflection, nonrigid matrices, and nonfinite components", () => {
  for (const rotation of [
    [[1, 0, 0], [0, 1, 0], [0, 0, -1]], [[2, 0, 0], [0, 1, 0], [0, 0, 1]],
    [[NaN, 0, 0], [0, 1, 0], [0, 0, 1]],
  ]) assert.throws(() => createFloorProjectionContext({ ...context,
    cameraToBase: { ...context.cameraToBase, rotation: rotation as unknown as FloorProjectionContext["cameraToBase"]["rotation"] } }), /rotation|finite/);
  assert.throws(() => createFloorProjectionContext({ ...context,
    cameraToBase: { ...context.cameraToBase, translationUnits: "mm" as "meters" } }), /units/);
  assert.throws(() => createFloorProjectionContext({ ...context,
    cameraToBase: { ...context.cameraToBase, translation: [0, 0, Infinity] } }), /finite/);
  assert.throws(() => createFloorProjectionContext({ ...context,
    floorPlane: { ...context.floorPlane, normal: [0, 0, 2] } }), /unit length/);
  assert.throws(() => createFloorProjectionContext({ ...context,
    floorPlane: { ...context.floorPlane, offset: NaN } }), /finite/);
  assert.throws(() => createFloorProjectionContext({ ...context,
    floorPlane: { ...context.floorPlane, units: "mm" as "meters" } }), /units/);
});

test("horizon, behind-camera, camera-on-plane, and over-range intersections reject", () => {
  assert.throws(() => project(center, observation, calibration, { ...context,
    cameraToBase: { ...context.cameraToBase, rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]] } }), /behind camera/);
  assert.throws(() => project(center, observation, calibration, { ...context,
    cameraToBase: { ...context.cameraToBase, rotation: [[0, 0, 1], [-1, 0, 0], [0, -1, 0]] } }), /horizon/);
  assert.throws(() => project(center, observation, calibration, { ...context,
    cameraToBase: { ...context.cameraToBase, translation: [0, 0, 0] } }), /origin/);
  assert.throws(() => project(center, observation, calibration, context, { ...policy, maxRangeMeters: 0.99 }), /range/);
  for (const badPolicy of [{ ...policy, maxRangeMeters: Infinity }, { ...policy, minIncidenceCosine: 0 },
    { ...policy, minIncidenceCosine: 1.1 }, { ...policy, maxUncertaintyMeters: -1 }]) {
    assert.throws(() => project(center, observation, calibration, context, badPolicy));
  }
});

test("uncertainty propagates measured pose/plane/ray bounds and enforces budget and robust incidence", () => {
  const f = { ...context, uncertainty: { ...context.uncertainty, positionErrorMeters: 0.01, planeOffsetErrorMeters: 0.02 } };
  const result = project(center, observation, calibration, f);
  close(result.uncertaintyRadiusMeters, 0.04);
  assert.throws(() => project(center, observation, calibration, f, { ...policy, maxUncertaintyMeters: 0.039 }), /uncertainty budget/);
  const angular = project(center, observation, { ...calibration, rayAngularErrorRad: 0.001 }, {
    ...f, uncertainty: { ...f.uncertainty, pixelRayAngularErrorRad: 0.001, poseAngularErrorRad: 0.001, planeNormalErrorRad: 0.001 },
  });
  assert.ok(angular.uncertaintyRadiusMeters > result.uncertaintyRadiusMeters);
  for (const positionErrorMeters of [-1, Infinity, NaN]) {
    assert.throws(() => project(center, observation, calibration, { ...context,
      uncertainty: { ...context.uncertainty, positionErrorMeters } }), /uncertainty/);
  }
  assert.throws(() => project(center, observation, calibration, { ...context,
    uncertainty: { ...context.uncertainty, planeOffsetErrorMeters: 1 } }), /origin/);
  assert.throws(() => project(center, observation, calibration, { ...context,
    uncertainty: { ...context.uncertainty, poseAngularErrorRad: 1 } }), /horizon/);
});

test("bounded JPEG header parsing extracts actual encoded resolution without transport", () => {
  // Synthetic header fixture, not a camera image or decoder fixture.
  const jpeg = Uint8Array.from([
    0xff, 0xd8, 0xff, 0xe0, 0, 4, 0, 0,
    0xff, 0xc0, 0, 11, 8, 0, 101, 0, 201, 1, 1, 0x11, 0,
    0xff, 0xda, 0, 8, 1, 1, 0, 0, 63, 0, 0xff, 0xd9,
  ]);
  assert.deepEqual(jpegImageDimensions(jpeg), { width: 201, height: 101 });
  const progressive = jpeg.slice(); progressive[9] = 0xc2;
  assert.deepEqual(jpegImageDimensions(progressive), { width: 201, height: 101 });
  assert.throws(() => jpegImageDimensions(Uint8Array.from([255, 216, 255, 217])), /scan header/);
  assert.throws(() => jpegImageDimensions(jpeg.slice(0, -1)), /incomplete JPEG/);
  const zeroWidth = jpeg.slice(); zeroWidth[17] = 0;
  assert.throws(() => jpegImageDimensions(zeroWidth), /dimensions/);
  const truncated = jpeg.slice(); truncated[11] = 255;
  assert.throws(() => jpegImageDimensions(truncated), /segment length/);
  const badPrecision = jpeg.slice(); badPrecision[12] = 12;
  assert.throws(() => jpegImageDimensions(badPrecision), /dimensions header/);
});
