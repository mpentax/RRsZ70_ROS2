/** Pure camera evidence contracts. No transport, stock calibration, or hardware access. */
export type ImageDimensions = Readonly<{ width: number; height: number }>;
export type CameraSource = Readonly<{ sourceId: string; lensId: string | null }>;
export type CameraFrameIdentity = Readonly<{
  source: CameraSource; streamId: string; sessionId: string; frameId: string;
}>;
export type EvidenceProvenance = Readonly<{ evidenceId: string; measuredAt: number }>;
export type LensVerification = Readonly<{ status: "unknown" }> |
  Readonly<{ status: "verified"; provenance: EvidenceProvenance }>;
export type CaptureProvenance = Readonly<{
  method: "synchronized-camera-clock" | "hardware-trigger";
  evidenceId: string; clockUncertaintyMs: number;
}>;
export type CameraObservation = Readonly<{
  identity: CameraFrameIdentity; dimensions: ImageDimensions;
  receivedAt: number; capturedAt: number | null;
  captureProvenance: CaptureProvenance | null; lensVerification: LensVerification;
}>;
export type Vector3 = readonly [number, number, number];
export type Matrix3 = readonly [Vector3, Vector3, Vector3];
export type CameraDistortion = Readonly<{ model: "none" }> | Readonly<{
  model: "brown-conrady"; k1: number; k2: number; k3: number; p1: number; p2: number;
  /** Measured validity domain in undistorted normalized camera coordinates. */
  maxRadiusNormalized: number;
}>;
export type CameraCalibration = Readonly<{
  calibrationId: string; source: CameraSource; dimensions: ImageDimensions;
  intrinsics: Readonly<{ fx: number; fy: number; cx: number; cy: number; units: "pixels" }>;
  distortion: CameraDistortion; provenance: EvidenceProvenance;
  validFrom: number; validUntil: number;
  /** Measured upper bound on intrinsic/distortion ray error across this resolution. */
  rayAngularErrorRad: number;
}>;
export type FloorProjectionContext = Readonly<{
  identity: CameraFrameIdentity; baseFrameId: string;
  cameraToBase: Readonly<{
    /** Optical camera axes: x right, y down, z forward. Base: x forward, y left, z up. */
    rotation: Matrix3; translation: Vector3; translationUnits: "meters";
    provenance: EvidenceProvenance;
  }>;
  floorPlane: Readonly<{
    /** Unit normal n, plane n dot point + offset = 0, in the named base frame. */
    normal: Vector3; offset: number; units: "meters"; provenance: EvidenceProvenance;
  }>;
  /** Certified bounds for this frame, including pixel localization and moving-mount pose. */
  uncertainty: Readonly<{
    pixelRayAngularErrorRad: number; poseAngularErrorRad: number; positionErrorMeters: number;
    planeNormalErrorRad: number; planeOffsetErrorMeters: number;
    provenance: EvidenceProvenance;
  }>;
}>;
export type MetricCameraPolicy = Readonly<{ now: number; maxCaptureAgeMs: number }>;
export type FloorProjectionPolicy = MetricCameraPolicy & Readonly<{
  maxRangeMeters: number; maxUncertaintyMeters: number; minIncidenceCosine: number;
}>;
export type BaseFloorProjection = Readonly<{
  identity: CameraFrameIdentity; calibrationId: string; baseFrameId: string;
  pixel: Readonly<{ x: number; y: number }>; point: Vector3; units: "meters";
  capturedAt: number; rangeMeters: number;
  /** Conservative Euclidean error radius, conditional on the supplied measured bounds. */
  uncertaintyRadiusMeters: number;
}>;

function requireValue(condition: boolean, message: string): asserts condition {
  if (!condition) throw new Error(`Camera observation: ${message}.`);
}
function finite(value: number, name: string): number {
  requireValue(typeof value === "number" && Number.isFinite(value), `${name} must be finite`);
  return value;
}
function nonnegative(value: number, name: string): number {
  requireValue(finite(value, name) >= 0, `${name} must be nonnegative`); return value;
}
function positive(value: number, name: string): number {
  requireValue(finite(value, name) > 0, `${name} must be positive`); return value;
}
function timestamp(value: number, name: string): number {
  requireValue(Number.isSafeInteger(nonnegative(value, name)), `${name} must be Unix milliseconds`); return value;
}
function id(value: string): string {
  requireValue(typeof value === "string" && /^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$/.test(value), "invalid opaque identity");
  return value;
}
function source(value: CameraSource): CameraSource {
  return Object.freeze({ sourceId: id(value.sourceId), lensId: value.lensId === null ? null : id(value.lensId) });
}
function dimensions(value: ImageDimensions): ImageDimensions {
  for (const size of [value.width, value.height]) {
    requireValue(Number.isSafeInteger(size) && size > 0, "image dimensions must be positive finite integers");
  }
  return Object.freeze({ width: value.width, height: value.height });
}
function identity(value: CameraFrameIdentity): CameraFrameIdentity {
  return Object.freeze({ source: source(value.source), streamId: id(value.streamId),
    sessionId: id(value.sessionId), frameId: id(value.frameId) });
}
function provenance(value: EvidenceProvenance): EvidenceProvenance {
  return Object.freeze({ evidenceId: id(value.evidenceId), measuredAt: timestamp(value.measuredAt, "measuredAt") });
}
function vector(value: Vector3): Vector3 {
  requireValue(Array.isArray(value) && value.length === 3, "expected three-vector");
  return Object.freeze(value.map(v => finite(v, "vector component"))) as unknown as Vector3;
}
function dot(a: Vector3, b: Vector3): number { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function norm(a: Vector3): number { return Math.hypot(...a); }
function sameSource(a: CameraSource, b: CameraSource): boolean {
  return a.sourceId === b.sourceId && a.lensId === b.lensId;
}
function sameFrame(a: CameraFrameIdentity, b: CameraFrameIdentity): boolean {
  return sameSource(a.source, b.source) && a.streamId === b.streamId &&
    a.sessionId === b.sessionId && a.frameId === b.frameId;
}
function angle(value: number): number {
  requireValue(nonnegative(value, "angular uncertainty") < Math.PI / 2, "angular uncertainty must be below pi/2 radians");
  return value;
}

/** Copies and deeply freezes evidence; invalid or contradictory provenance is rejected. */
export function createCameraObservation(value: CameraObservation): CameraObservation {
  const frame = identity(value.identity);
  const receivedAt = timestamp(value.receivedAt, "receivedAt");
  const capturedAt = value.capturedAt === null ? null : timestamp(value.capturedAt, "capturedAt");
  requireValue(capturedAt === null || capturedAt <= receivedAt, "capture is later than receipt");
  requireValue((capturedAt === null) === (value.captureProvenance === null), "capture time requires capture provenance");
  let captureProvenance: CaptureProvenance | null = null;
  if (value.captureProvenance !== null) {
    const p = value.captureProvenance;
    requireValue(p.method === "synchronized-camera-clock" || p.method === "hardware-trigger", "unsupported capture clock provenance");
    captureProvenance = Object.freeze({ method: p.method, evidenceId: id(p.evidenceId),
      clockUncertaintyMs: nonnegative(p.clockUncertaintyMs, "clock uncertainty") });
  }
  const lens = value.lensVerification;
  requireValue(lens.status === "unknown" || lens.status === "verified", "invalid lens verification");
  let lensVerification: LensVerification = Object.freeze({ status: "unknown" });
  if (lens.status === "verified") {
    const p = provenance(lens.provenance);
    requireValue(frame.source.lensId !== null && p.measuredAt <= (capturedAt ?? receivedAt), "lens verification does not precede observation");
    lensVerification = Object.freeze({ status: "verified", provenance: p });
  }
  return Object.freeze({ identity: frame, dimensions: dimensions(value.dimensions), receivedAt, capturedAt,
    captureProvenance, lensVerification });
}

/** Use immediately after actual byte receipt. Local frameId is a receipt identity, not a sensor sequence. */
export function recordCameraFrameReceipt(value: Readonly<{
  identity: CameraFrameIdentity; dimensions: ImageDimensions; receivedAt: number;
}>): CameraObservation {
  return createCameraObservation({ identity: value.identity, dimensions: value.dimensions, receivedAt: value.receivedAt,
    capturedAt: null, captureProvenance: null, lensVerification: { status: "unknown" } });
}

/** Reads JPEG header dimensions only; does not decode pixels or prove capture freshness. */
export function jpegImageDimensions(jpeg: Uint8Array): ImageDimensions {
  requireValue(jpeg instanceof Uint8Array && jpeg.length >= 4 && jpeg[0] === 0xff && jpeg[1] === 0xd8 &&
    jpeg[jpeg.length - 2] === 0xff && jpeg[jpeg.length - 1] === 0xd9, "incomplete JPEG");
  let offset = 2;
  let found: ImageDimensions | null = null;
  while (offset < jpeg.length - 2) {
    requireValue(jpeg[offset++] === 0xff, "invalid JPEG marker");
    while (jpeg[offset] === 0xff) offset++;
    const marker = jpeg[offset++];
    requireValue(marker !== undefined && marker !== 0 && marker !== 0xd8 && marker !== 0xd9, "invalid JPEG marker");
    if (marker === 0x01 || (marker >= 0xd0 && marker <= 0xd7)) continue;
    requireValue(offset + 2 <= jpeg.length - 2, "truncated JPEG segment");
    const length = jpeg[offset] * 256 + jpeg[offset + 1];
    requireValue(length >= 2 && offset + length <= jpeg.length - 2, "invalid JPEG segment length");
    if ([0xc0, 0xc1, 0xc2].includes(marker)) {
      requireValue(!found && length >= 8 && jpeg[offset + 2] === 8 &&
        jpeg[offset + 7] > 0 && length === 8 + 3 * jpeg[offset + 7], "invalid or unsupported JPEG dimensions header");
      found = dimensions({ height: jpeg[offset + 3] * 256 + jpeg[offset + 4],
        width: jpeg[offset + 5] * 256 + jpeg[offset + 6] });
    }
    if (marker === 0xda) {
      requireValue(found !== null, "JPEG has no supported dimensions header"); return found;
    }
    offset += length;
  }
  throw new Error("Camera observation: JPEG has no scan header.");
}

export function createCameraCalibration(value: CameraCalibration): CameraCalibration {
  const s = source(value.source);
  requireValue(s.lensId !== null, "calibration requires a physical lens identity");
  const size = dimensions(value.dimensions);
  const i = value.intrinsics;
  requireValue(i.units === "pixels", "intrinsics units must be pixels");
  positive(i.fx, "fx"); positive(i.fy, "fy");
  requireValue(nonnegative(i.cx, "cx") <= size.width - 1 && nonnegative(i.cy, "cy") <= size.height - 1,
    "principal point outside calibrated resolution");
  const d = value.distortion;
  requireValue(d.model === "none" || d.model === "brown-conrady", "unsupported distortion model");
  let distortion: CameraDistortion = Object.freeze({ model: "none" });
  if (d.model === "brown-conrady") {
    for (const coefficient of [d.k1, d.k2, d.k3, d.p1, d.p2]) finite(coefficient, "distortion coefficient");
    const r = positive(d.maxRadiusNormalized, "distortion validity radius");
    // Sufficient (conservative) Jacobian bound: mapping is injective on the calibrated disk.
    const bound = 3 * Math.abs(d.k1) * r ** 2 + 5 * Math.abs(d.k2) * r ** 4 +
      7 * Math.abs(d.k3) * r ** 6 + 12 * r * (Math.abs(d.p1) + Math.abs(d.p2));
    requireValue(Number.isFinite(bound) && bound < 1, "distortion domain cannot guarantee unique inversion");
    distortion = Object.freeze({ model: d.model, k1: d.k1, k2: d.k2, k3: d.k3, p1: d.p1, p2: d.p2, maxRadiusNormalized: r });
  }
  const p = provenance(value.provenance);
  const validFrom = timestamp(value.validFrom, "validFrom"), validUntil = timestamp(value.validUntil, "validUntil");
  requireValue(p.measuredAt <= validFrom && validFrom < validUntil, "invalid calibration provenance or validity interval");
  return Object.freeze({ calibrationId: id(value.calibrationId), source: s, dimensions: size,
    intrinsics: Object.freeze({ fx: i.fx, fy: i.fy, cx: i.cx, cy: i.cy, units: "pixels" }), distortion,
    provenance: p, validFrom, validUntil, rayAngularErrorRad: angle(value.rayAngularErrorRad) });
}

export function createFloorProjectionContext(value: FloorProjectionContext): FloorProjectionContext {
  const t = value.cameraToBase;
  requireValue(t.translationUnits === "meters", "transform translation units must be meters");
  requireValue(Array.isArray(t.rotation) && t.rotation.length === 3, "rotation must be 3x3");
  const rotation = Object.freeze(t.rotation.map(vector)) as unknown as Matrix3;
  for (let row = 0; row < 3; row++) for (let col = 0; col < 3; col++) {
    requireValue(Math.abs(dot(rotation[row], rotation[col]) - (row === col ? 1 : 0)) < 1e-9, "rotation must be orthonormal");
  }
  const [a, b, c] = rotation;
  const determinant = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
  requireValue(Math.abs(determinant - 1) < 1e-9, "rotation must be proper, not reflected");
  const plane = value.floorPlane;
  requireValue(plane.units === "meters", "floor plane units must be meters");
  const normal = vector(plane.normal);
  requireValue(Math.abs(norm(normal) - 1) < 1e-9, "floor plane normal must be unit length");
  const u = value.uncertainty;
  return Object.freeze({ identity: identity(value.identity), baseFrameId: id(value.baseFrameId),
    cameraToBase: Object.freeze({ rotation, translation: vector(t.translation), translationUnits: "meters", provenance: provenance(t.provenance) }),
    floorPlane: Object.freeze({ normal, offset: finite(plane.offset, "plane offset"), units: "meters", provenance: provenance(plane.provenance) }),
    uncertainty: Object.freeze({ pixelRayAngularErrorRad: angle(u.pixelRayAngularErrorRad), poseAngularErrorRad: angle(u.poseAngularErrorRad),
      positionErrorMeters: nonnegative(u.positionErrorMeters, "position uncertainty"), planeNormalErrorRad: angle(u.planeNormalErrorRad),
      planeOffsetErrorMeters: nonnegative(u.planeOffsetErrorMeters, "plane offset uncertainty"), provenance: provenance(u.provenance) }) });
}

/** Receipt recency alone never admits metric evidence. Revalidates even plain/JSON objects. */
export function assertMetricCameraObservation(observation: CameraObservation, calibration: CameraCalibration,
  policy: MetricCameraPolicy): void {
  const o = createCameraObservation(observation), c = createCameraCalibration(calibration);
  const now = timestamp(policy.now, "now");
  nonnegative(policy.maxCaptureAgeMs, "maximum capture age");
  requireValue(o.capturedAt !== null && o.captureProvenance !== null, "capture time is unknown");
  requireValue(o.lensVerification.status === "verified", "lens is unverified");
  requireValue(sameSource(o.identity.source, c.source), "calibration source/lens mismatch");
  requireValue(o.dimensions.width === c.dimensions.width && o.dimensions.height === c.dimensions.height, "calibration resolution mismatch");
  const error = o.captureProvenance.clockUncertaintyMs;
  requireValue(o.lensVerification.provenance.measuredAt <= o.capturedAt - error, "lens verification postdates earliest capture");
  requireValue(o.receivedAt <= now && o.capturedAt + error <= now, "future observation");
  requireValue(now - o.capturedAt + error <= policy.maxCaptureAgeMs, "stale capture");
  requireValue(o.capturedAt - error >= c.validFrom && o.capturedAt + error <= c.validUntil, "capture outside calibration validity");
}

function normalizedPixel(xd: number, yd: number, d: CameraDistortion): readonly [number, number] {
  if (d.model === "none") return [xd, yd];
  let x = xd, y = yd;
  const initialRadius = Math.hypot(x, y);
  if (initialRadius > d.maxRadiusNormalized) { x *= d.maxRadiusNormalized / initialRadius; y *= d.maxRadiusNormalized / initialRadius; }
  for (let iteration = 0; iteration < 50; iteration++) {
    const r2 = x * x + y * y, r4 = r2 * r2;
    const radial = 1 + d.k1 * r2 + d.k2 * r4 + d.k3 * r4 * r2;
    const ex = x * radial + 2 * d.p1 * x * y + d.p2 * (r2 + 2 * x * x) - xd;
    const ey = y * radial + d.p1 * (r2 + 2 * y * y) + 2 * d.p2 * x * y - yd;
    if (Math.hypot(ex, ey) <= 1e-12) {
      requireValue(Math.hypot(x, y) <= d.maxRadiusNormalized, "pixel outside distortion calibration domain"); return [x, y];
    }
    const slope = d.k1 + 2 * d.k2 * r2 + 3 * d.k3 * r4;
    const a = radial + 2 * x * x * slope + 2 * d.p1 * y + 6 * d.p2 * x;
    const b = 2 * x * y * slope + 2 * d.p1 * x + 2 * d.p2 * y;
    const c = radial + 2 * y * y * slope + 6 * d.p1 * y + 2 * d.p2 * x;
    const det = a * c - b * b;
    requireValue(Number.isFinite(det) && det > 0, "distortion inversion is singular");
    x -= (c * ex - b * ey) / det; y -= (a * ey - b * ex) / det;
    requireValue(Number.isFinite(x) && Number.isFinite(y) && Math.hypot(x, y) <= d.maxRadiusNormalized + 1e-12,
      "pixel outside distortion calibration domain");
  }
  throw new Error("Camera observation: distortion inversion did not converge.");
}

export function projectPixelToBase(observation: CameraObservation, calibration: CameraCalibration,
  context: FloorProjectionContext, pixel: Readonly<{ x: number; y: number }>, policy: FloorProjectionPolicy): BaseFloorProjection {
  const o = createCameraObservation(observation), c = createCameraCalibration(calibration), f = createFloorProjectionContext(context);
  assertMetricCameraObservation(o, c, policy);
  requireValue(sameFrame(o.identity, f.identity), "projection frame mismatch");
  for (const p of [f.cameraToBase.provenance, f.floorPlane.provenance, f.uncertainty.provenance]) {
    requireValue(p.measuredAt <= o.capturedAt! - o.captureProvenance!.clockUncertaintyMs, "projection provenance postdates capture");
  }
  positive(policy.maxRangeMeters, "maximum range"); nonnegative(policy.maxUncertaintyMeters, "maximum uncertainty");
  requireValue(positive(policy.minIncidenceCosine, "minimum incidence") <= 1, "minimum incidence exceeds one");
  const x = finite(pixel.x, "pixel x"), y = finite(pixel.y, "pixel y");
  requireValue(x >= 0 && x <= o.dimensions.width - 1 && y >= 0 && y <= o.dimensions.height - 1, "pixel outside image");
  const i = c.intrinsics;
  const [rx, ry] = normalizedPixel((x - i.cx) / i.fx, (y - i.cy) / i.fy, c.distortion);
  const length = Math.hypot(rx, ry, 1);
  const ray: Vector3 = [rx / length, ry / length, 1 / length];
  const direction = f.cameraToBase.rotation.map(row => dot(row, ray)) as unknown as Vector3;
  const origin = f.cameraToBase.translation, n = f.floorPlane.normal, u = f.uncertainty;
  const rayError = 2 * Math.sin(angle(c.rayAngularErrorRad + u.pixelRayAngularErrorRad + u.poseAngularErrorRad) / 2);
  const normalError = 2 * Math.sin(u.planeNormalErrorRad / 2);
  const denominator = dot(n, direction), height = dot(n, origin) + f.floorPlane.offset;
  const denominatorError = normalError + rayError;
  const heightError = u.positionErrorMeters + u.planeOffsetErrorMeters + normalError * (norm(origin) + u.positionErrorMeters);
  requireValue(Math.abs(denominator) - denominatorError >= policy.minIncidenceCosine, "ray parallel or uncertain near floor horizon");
  requireValue(Math.abs(height) > heightError, "camera origin on or uncertain across floor plane");
  const distance = -height / denominator;
  requireValue(Number.isFinite(distance) && distance > 0, "floor intersection behind camera");
  const distanceError = (heightError + distance * denominatorError) / (Math.abs(denominator) - denominatorError);
  const uncertaintyRadiusMeters = u.positionErrorMeters + distanceError + (distance + distanceError) * rayError;
  requireValue(Number.isFinite(uncertaintyRadiusMeters) && distance + distanceError <= policy.maxRangeMeters, "floor projection exceeds range");
  requireValue(uncertaintyRadiusMeters <= policy.maxUncertaintyMeters, "floor projection exceeds uncertainty budget");
  const point = vector(origin.map((v, index) => v + distance * direction[index]) as unknown as Vector3);
  return Object.freeze({ identity: o.identity, calibrationId: c.calibrationId, baseFrameId: f.baseFrameId,
    pixel: Object.freeze({ x, y }), point, units: "meters", capturedAt: o.capturedAt!, rangeMeters: distance, uncertaintyRadiusMeters });
}
