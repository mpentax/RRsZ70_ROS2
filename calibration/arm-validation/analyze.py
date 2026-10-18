"""Offline marker-pose and settling analysis. Never contacts the robot.

The default ROI excludes the floor boards; visually verify a newly selected
arm marker before accepting it. Marker-to-wrist offsets are NOT assumed zero.
From the repository root: pixi run python calibration/arm-validation/analyze.py --frame1 a.jpg --frame2 b.jpg --output pair.json
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
CALIBRATION = HERE.parent / "front-camera" / "front_camera.json"


def intrinsics(cal):
    k = np.array([[cal["f_px"], 0, cal["cx"]],
                  [0, cal["f_px"], cal["cy"]], [0, 0, 1]], dtype=np.float64)
    d = np.array([cal["k1"], cal["k2"], 0, 0, 0], dtype=np.float64)
    return k, d


def marker_pose(corners, side, k, d):
    half = side / 2
    obj = np.array([[-half, half, 0], [half, half, 0],
                    [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        obj, corners.astype(np.float64), k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    poses = []
    if ok:
        for rvec, tvec in zip(rvecs, tvecs):
            if tvec.reshape(3)[2] <= 0:
                continue
            proj, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
            rms = float(np.sqrt(np.mean(np.sum(
                (proj.reshape(4, 2) - corners) ** 2, axis=1))))
            poses.append({"rvec": rvec.reshape(3).tolist(),
                          "tvec_m": tvec.reshape(3).tolist(),
                          "reprojection_rms_px": rms})
    poses.sort(key=lambda p: p["reprojection_rms_px"])
    if not poses:
        return None
    result = dict(poses[0])
    result["alternative_solutions"] = poses[1:]
    return result


def detect(path, cal, roi, side, compute_pose=True):
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError("Cannot read image: " + str(path))
    h, w = image.shape[:2]
    if (w, h) != (cal["width"], cal["height"]):
        raise ValueError("Image dimensions do not match the calibration")
    x0, y0, x1, y1 = roi
    if not (0 <= x0 < x1 <= w and 0 <= y0 < y1 <= h):
        raise ValueError("ROI is outside the image")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    k, d = intrinsics(cal)
    detections = []
    # Reuse front-camera/detect.py parameters, including its small-marker pass.
    for scale, win, clahe in [(1, 23, False), (2, 53, True)]:
        g = cv2.createCLAHE(3.0, (8, 8)).apply(gray) if clahe else gray.copy()
        mask = np.zeros_like(g, dtype=bool)
        mask[y0:y1, x0:x1] = True
        g[~mask] = 255
        if scale != 1:
            g = cv2.resize(g, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        p = cv2.aruco.DetectorParameters()
        p.adaptiveThreshWinSizeMax = win
        p.adaptiveThreshWinSizeStep = 6
        p.polygonalApproxAccuracyRate = 0.03
        p.errorCorrectionRate = 0.8
        p.minMarkerPerimeterRate = 0.01
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        cs, ids, _ = cv2.aruco.ArucoDetector(dictionary, p).detectMarkers(g)
        if ids is None:
            continue
        for corners, ident in zip(cs, ids.reshape(-1)):
            corners = corners.reshape(4, 2).astype(np.float64) / scale
            edges = np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)
            if edges.min() < 8 or abs(cv2.contourArea(corners.astype(np.float32))) < 64:
                continue
            centre = corners.mean(axis=0)
            if any(v["id"] == int(ident) and
                   np.linalg.norm(np.array(v["centre_px"]) - centre) < 8
                   for v in detections):
                continue
            pose = marker_pose(corners, side, k, d) if compute_pose else None
            record = {"id": int(ident), "corners_px": corners.tolist(),
                      "centre_px": centre.tolist(), "pose": pose}
            if pose:
                record.update({key: pose[key] for key in ("rvec", "tvec_m")})
            detections.append(record)
    return image, detections


def analyse_pair(first, second, marker_id):
    a = [x for x in first if marker_id is None or x["id"] == marker_id]
    b = [x for x in second if marker_id is None or x["id"] == marker_id]
    if not a or not b:
        return {"status": "marker_not_detected", "accepted": False, "marker": None}
    if len(a) != 1 or len(b) != 1 or a[0]["id"] != b[0]["id"]:
        return {"status": "ambiguous_marker_identity", "accepted": False, "marker": None}
    shift = float(np.linalg.norm(np.array(a[0]["centre_px"]) - b[0]["centre_px"]))
    valid_pose = all(x["pose"] is not None and
                     x["pose"]["reprojection_rms_px"] <= 2 for x in [a[0], b[0]])
    accepted = shift <= 1 and valid_pose
    return {"status": "settled" if accepted else "discard_unsettled_or_invalid_pose",
            "accepted": accepted, "centre_shift_px": shift,
            "marker": b[0] if accepted else None}


# Report inputs are snapshots; no model state is used as measurement evidence.
DIRECTIONS = {1: "up", 2: "down", 3: "forward", 4: "back"}
OPPOSITE = {1: 2, 2: 1, 3: 4, 4: 3}


def local_path(value, base):
    path = Path(value)
    # Preserve recorded absolute paths verbatim in the archive while resolving
    # this one approved checkout relocation; never search arbitrary ancestors.
    legacy = HERE.parents[1].with_name('RoboRockZ70-Connector') / 'calibration/arm-validation'
    if path.is_absolute() and path.is_relative_to(legacy):
        path = HERE / path.relative_to(legacy)
    path = (base / path).resolve() if not path.is_absolute() else path.resolve()
    if not path.is_relative_to(HERE):
        raise ValueError("Report evidence/output must stay under " + str(HERE))
    return path


def relative_angle(a, b):
    """Magnitude of R_after R_before^T, not subtraction of Rodrigues vectors."""
    ra = cv2.Rodrigues(np.asarray(a, dtype=np.float64).reshape(3))[0]
    rb = cv2.Rodrigues(np.asarray(b, dtype=np.float64).reshape(3))[0]
    delta = cv2.Rodrigues(rb @ ra.T)[0]
    return float(np.degrees(np.linalg.norm(delta)))


def stats(values):
    if not values:
        return ["unavailable", "unavailable", 0]
    return [float(np.mean(values)),
            float(np.std(values, ddof=1)) if len(values) > 1 else "unavailable", len(values)]


def table(headers, rows):
    def cell(value):
        if isinstance(value, (float, np.floating)):
            value = f"{value:.6g}"
        return str(value).replace("|", "\\|").replace("\n", " ")
    return "\n".join(["| " + " | ".join(map(cell, headers)) + " |",
                      "| " + " | ".join(["---"] * len(headers)) + " |"] +
                     ["| " + " | ".join(map(cell, row)) + " |" for row in rows])


def command_key(record):
    params = record.get("params") or {}
    if record.get("method") == "app_arm_direction_move":
        direction = params.get("direction")
        return ("translation", direction) if direction in DIRECTIONS else None
    if record.get("method") == "app_arm_move":
        moves = params.get("m")
        if isinstance(moves, list) and len(moves) == 3:
            active = [(i + 3, x) for i, x in enumerate(moves) if x != 0]
            if len(active) == 1 and active[0][1] in (1, 2):
                return ("rotation", *active[0])
    return None


def measurement_phase(record):
    if record.get("measurement_sample") is False or record.get("metric_measurement") is False:
        return False
    if record.get("per_step_capture_omitted"):
        return False
    phase = str(record.get("phase", "")).lower()
    return not any(word in phase for word in ("setup", "search", "find_gripper", "attachment", "inspection"))


def accepted_pose(record, session, cal, side_override, base):
    """Require explicit acceptance, physical planarity and a settled metric pose.

    A later replacement cannot rehabilitate an earlier bowed target. Analysis
    side lengths are checked per sample, never silently rescaled from metadata.
    """
    inspection = record.get("inspection_metrics") or {}
    reason = record.get("pose_measurement_rejected_reason")
    if reason:
        return None, str(reason)
    if (record.get("nonplanar") is True or inspection.get("printed_region_visually_bowed") is True
            or record.get("marker_rigidity_verified") is False
            or inspection.get("marker_rigidity_verified") is False):
        return None, "nonplanar/unverified rigid target (explicit rejection)"
    analysis = record.get("settling_analysis") or record.get("analysis") or {}
    if record.get("settling_analysis_file"):
        try:
            analysis = json.loads(local_path(record["settling_analysis_file"], base).read_bytes())
        except (OSError, ValueError, TypeError) as error:
            return None, "analysis unavailable: " + str(error)
    if not isinstance(analysis, dict):
        return None, "invalid analysis object"
    if record.get("accepted_marker_measurement") is False or record.get("accepted") is False:
        return None, record.get("settling_status", "explicitly rejected")
    accepted = (record.get("accepted_marker_measurement") is True
                or record.get("accepted") is True or analysis.get("accepted") is True)
    if not accepted:
        return None, record.get("settling_status", "no explicitly accepted marker pose")
    rigid = record.get("marker_rigidity_verified", session.get("marker_rigidity_verified"))
    if rigid is not True:
        return None, "physical marker rigidity/planarity not verified"
    if analysis and (analysis.get("accepted") is not True
                     or analysis.get("metric_pose_computed") is False):
        return None, "settling analysis rejected or detection-only"
    frames = record.get("frame_pair")
    if analysis and frames:
        if len(frames) != 2 or any(local_path(frames[i], base) !=
                                  local_path(analysis.get(f"frame{i + 1}", ""), base)
                                  for i in range(2)):
            return None, "analysis frame-pair provenance mismatch"
    size = analysis.get("marker_side_m", record.get("marker_side_m", session.get("marker_side_m")))
    if size is None or not np.isfinite(size) or size <= 0:
        return None, "known black marker side unavailable"
    if side_override is not None and not np.isclose(size, side_override, rtol=0, atol=1e-9):
        return None, "marker side differs from explicit report side; not rescaled"
    marker = analysis.get("marker") or record.get("marker")
    if not isinstance(marker, dict) or marker.get("id") is None:
        return None, "accepted marker identity unavailable"
    wanted = record.get("marker_id", session.get("marker_id"))
    if wanted is not None and marker["id"] != wanted:
        return None, "marker identity mismatch"
    settling = analysis.get("centre_shift_px", record.get("centre_shift_px"))
    if settling is None and record.get("frame_pair_settling_verified") is not True:
        return None, "two-frame settling evidence unavailable"
    if settling is not None and (not np.isfinite(settling) or settling > 1):
        return None, "unsettled marker centre"
    pose = marker.get("pose") or marker
    if marker.get("corners_px") is not None:
        corners = np.asarray(marker["corners_px"], dtype=np.float64)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            return None, "invalid marker corners"
        pose = marker_pose(corners, size, *intrinsics(cal))
    if pose is None:
        return None, "metric planar pose fit failed"
    try:
        t = np.asarray(pose["tvec_m"], dtype=np.float64).reshape(3)
        r = np.asarray(pose["rvec"], dtype=np.float64).reshape(3)
        rms = pose["reprojection_rms_px"]
        if not np.isfinite(t).all() or not np.isfinite(r).all() or t[2] <= 0 or not np.isfinite(rms) or rms > 2:
            return None, "invalid metric pose/reprojection RMS > 2 px"
    except (TypeError, KeyError, ValueError):
        return None, "metric pose/quality evidence unavailable"
    noise = None
    if analysis.get("markers_first") and analysis.get("markers_second"):
        first = [m for m in analysis["markers_first"] if m["id"] == marker["id"]]
        second = [m for m in analysis["markers_second"] if m["id"] == marker["id"]]
        if len(first) == len(second) == 1 and first[0].get("pose") and second[0].get("pose"):
            noise = relative_angle(first[0]["pose"]["rvec"], second[0]["pose"]["rvec"])
    return {"t": t, "r": r, "id": marker["id"], "side": size,
            "epoch": record.get("marker_epoch", session.get("marker_epoch")),
            "noise_deg": noise, "record": record}, None


def chassis_samples(records):
    """Only explicitly accepted poses in the same fixed floor-board frame.

    board_pose_before/after: {accepted, board_id, rvec, tvec_m,
    reprojection_rms_px}. No arm marker or app model can stand in for these.
    """
    samples, exclusions = [], []
    for number, record in records:
        if record.get("method") != "app_rc_move":
            continue
        try:
            params = record.get("params")
            if isinstance(params, list) and len(params) == 1:
                params = params[0]
            if not isinstance(params, dict):
                raise ValueError("pulse params must contain one command object")
            omega, velocity = float(params["omega"]), float(params["velocity"])
            duration = float(params["duration"]) / 1000
            if not np.isfinite([omega, velocity, duration]).all() or duration <= 0:
                raise ValueError("invalid pulse duration/velocity")
            if (omega == 0) == (velocity == 0):
                raise ValueError("not a pure rotation/translation pulse")
            a, b = record["board_pose_before"], record["board_pose_after"]
            if (a.get("accepted") is not True or b.get("accepted") is not True
                    or a.get("board_id") is None or a["board_id"] != b.get("board_id")):
                raise ValueError("no accepted common fixed-board poses")
            matrices, centres = [], []
            for pose in (a, b):
                r = np.asarray(pose["rvec"], dtype=np.float64).reshape(3)
                t = np.asarray(pose["tvec_m"], dtype=np.float64).reshape(3)
                rms = float(pose["reprojection_rms_px"])
                if not np.isfinite(r).all() or not np.isfinite(t).all() or t[2] <= 0 or not np.isfinite(rms) or rms > 2:
                    raise ValueError("invalid board pose/quality")
                matrix = cv2.Rodrigues(r)[0]
                matrices.append(matrix)
                centres.append(-matrix.T @ t)
            if omega:
                rotation = matrices[0] @ matrices[1].T
                # Express relative camera rotation in the BEFORE board frame.
                rotation = matrices[0].T @ rotation @ matrices[0]
                yaw = float(np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0])))
                gain, value, family, command = yaw / (omega * duration), yaw, "rotation", omega
            else:
                forward = matrices[0].T @ np.array([0., 0., 1.])
                forward[2] = 0
                if np.linalg.norm(forward) < 1e-6:
                    raise ValueError("camera-forward floor projection degenerate")
                forward /= np.linalg.norm(forward)
                distance = float(np.dot(centres[1] - centres[0], forward) * 1000)
                gain, value, family, command = distance / (velocity * duration), distance, "translation", velocity
            samples.append({"line": number, "family": family, "command": command,
                            "duration_s": duration, "gain": gain, "value": value})
        except (KeyError, TypeError, ValueError, AttributeError) as error:
            exclusions.append((number, record.get("t", "unavailable"), "app_rc_move", "chassis unavailable: " + str(error)))
    return samples, exclusions


def report(args, cal):
    log_path = local_path(args.log, Path.cwd())
    session_path = local_path(args.session, Path.cwd())
    session = json.loads(session_path.read_bytes())
    snapshot = log_path.read_bytes()
    records, parse_errors = [], []
    lines = snapshot.splitlines()
    for number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError("record is not an object")
            records.append((number, value))
        except (ValueError, UnicodeDecodeError) as error:
            kind = "truncated final line ignored" if number == len(lines) and not snapshot.endswith(b"\n") else "malformed record excluded"
            parse_errors.append((number, kind + ": " + str(error)))
    rejected, deltas, repeats = [], [], {}
    counts, eligible = {}, {}
    accepted_endpoints = 0
    baseline = pending = previous_key = None
    run = 0
    previous_number = 0
    for number, record in records:
        if any(previous_number < excluded < number for excluded, _ in parse_errors):
            baseline = pending = previous_key = None  # A malformed line may conceal a move.
        previous_number = number
        key = command_key(record)
        method = record.get("method", "")
        is_motion = method.startswith("app_arm_") or method in (
            "app_pick_up_enter_exit", "app_rc_move", "app_open_gripper", "app_grip")
        if key:
            counts[key] = counts.get(key, 0) + 1
            if measurement_phase(record):
                eligible[key] = eligible.get(key, 0) + 1
            if pending is not None:
                baseline = None  # No accepted observation between two commands.
                run += 1
            reversal = (previous_key is not None and key[0] == previous_key[0] and
                        ((key[0] == "translation" and OPPOSITE[key[1]] == previous_key[1]) or
                         (key[0] == "rotation" and key[1] == previous_key[1] and key[2] != previous_key[2])))
            if key != previous_key:
                run += 1
            pending = (number, record, key, reversal, run)
            previous_key = key
        elif is_motion or method == "physical_setup_observation":
            baseline = pending = previous_key = None
            run += 1
        has_evidence = any(record.get(field) is not None for field in (
            "frame_file", "frame_pair", "settling_analysis_file", "marker", "pose_measurement_rejected_reason"))
        if not has_evidence:
            continue
        try:
            pose, reason = accepted_pose(record, session, cal, args.marker_side_m, log_path.parent)
        except (ValueError, TypeError, KeyError) as error:
            pose, reason = None, "invalid evidence: " + str(error)
        if pose is None:
            rejected.append((number, record.get("t", "unavailable"), method, reason))
            baseline = pending = None
            run += 1
            continue
        accepted_endpoints += 1
        if pending is not None and baseline is not None:
            cmd_number, command, move, reversal, run_id = pending
            reply = command.get("rpc_reply") or {}
            if (reply.get("error") or reply.get("response") in ("timeout", "failed")
                    or reply.get("dispatch") in ("failed", "not_sent")):
                rejected.append((cmd_number, command.get("t", "unavailable"), method, "failed/ambiguous command delivery; not a step measurement"))
                baseline = pending = previous_key = None
                continue
            compatible = all(pose[field] == baseline[field] for field in ("id", "side", "epoch"))
            if not compatible:
                rejected.append((cmd_number, command.get("t", "unavailable"), method, "marker identity/size/epoch changed across move"))
            elif measurement_phase(command):
                dt = pose["t"] - baseline["t"]
                angle = relative_angle(baseline["r"], pose["r"])
                if move[0] == "rotation":
                    joint = f"M{move[1]}"
                    observes = command.get("marker_observes_joints", record.get("marker_observes_joints", session.get("marker_observes_joints")))
                    upstream = command.get("marker_upstream_of_joints", record.get("marker_upstream_of_joints", session.get("marker_upstream_of_joints"))) or []
                    threshold = max(1.0, 3 * max(pose["noise_deg"] or 0, baseline["noise_deg"] or 0))
                    if joint in upstream or (observes is not None and joint not in observes):
                        rejected.append((cmd_number, command.get("t", "unavailable"), method, joint + " unobservable: marker upstream of joint"))
                        baseline, pending = pose, None
                        run += 1
                        continue
                    if angle <= threshold:
                        rejected.append((cmd_number, command.get("t", "unavailable"), method, "low rotation unresolved (not a measured zero step); verify downstream attachment, limit and pose noise"))
                        baseline, pending = pose, None
                        run += 1
                        continue
                deltas.append({"line": cmd_number, "key": move, "value": float(np.linalg.norm(dt)) if move[0] == "translation" else angle,
                               "delta": dt, "angle": angle, "run": run_id, "reversal": reversal})
        repeat = record.get("repeatability")
        if isinstance(repeat, dict) and repeat.get("group") is not None and repeat.get("cycle") is not None:
            group = (str(repeat["group"]), pose["id"], pose["side"], pose["epoch"])
            repeats.setdefault(group, {}).setdefault(str(repeat["cycle"]), []).append(pose["t"])
        baseline, pending = pose, None
    chassis, chassis_rejected = chassis_samples(records)
    rejected.extend(chassis_rejected)
    sections = []
    def add(headers, rows):
        sections.append(table(headers, rows))
    add(["Snapshot", "Value"], [
        ["Scope", "PARTIAL snapshot; final acquisition is not complete"],
        ["Log", log_path], ["Session", session_path], ["Log bytes read once", len(snapshot)],
        ["Parsed records", len(records)], ["Malformed/truncated lines excluded", len(parse_errors)],
        ["Last parsed timestamp", records[-1][1].get("t", "unavailable") if records else "unavailable"],
        ["Session phase/status", str(session.get("phase", "unavailable")) + " / " + str(session.get("status", "unavailable"))],
        ["Report black-side filter (m)", args.marker_side_m if args.marker_side_m is not None else "per-sample metadata"],
        ["Accepted pose endpoints", accepted_endpoints],
        ["Accepted measured command deltas", len(deltas)], ["Rejected/unavailable evidence rows", len(rejected)],
        ["Chassis pulses logged / accepted", str(sum(r.get("method") == "app_rc_move" for _, r in records)) + " / " + str(len(chassis))],
        ["App model state", "Never used as measured pose or to infer offsets"],
        ["Intrinsics caveat", cal.get("note", "unavailable")]])
    keys = [("translation", d) for d in (3, 4, 2, 1)] + [("rotation", j, s) for j in (5, 4, 3) for s in (1, 2)]
    grouped = {key: [d["value"] for d in deltas if d["key"] == key] for key in keys}
    def label(key):
        return DIRECTIONS[key[1]] if key[0] == "translation" else f"M{key[1]} {'+' if key[2] == 1 else '-'}"
    add(["Measured command line", "Move", "Marker dx (m)", "Marker dy (m)", "Marker dz (m)", "Marker displacement norm (m)", "Relative rotation magnitude (deg)"],
        [[d["line"], label(d["key"]), *d["delta"].tolist(), float(np.linalg.norm(d["delta"])), d["angle"]] for d in deltas]
        or [["unavailable", "unavailable", "unavailable", "unavailable", "unavailable", "unavailable", "unavailable"]])
    for family, unit, prediction in [("translation", "m", 0.04), ("rotation", "deg", 10.0)]:
        rows = []
        for key in keys:
            if key[0] != family:
                continue
            mean, std, n = stats(grouped[key])
            rows.append([label(key), mean, std, n, mean / prediction if n else "unavailable",
                         "marker-centre displacement, NOT wrist displacement" if family == "translation" else "pure single-joint relative rotation magnitude; sign is command sign"])
        add(["Step " + family, "Mean (" + unit + ")", "Sample std (" + unit + ")", "n", "Measured / app-predicted", "Interpretation"], rows)
    add(["Coverage", "Commands logged (incl. setup)", "Measurement-phase commands", "Accepted deltas", "Minimum planned deltas", "Status"],
        [[label(k), counts.get(k, 0), eligible.get(k, 0), len(grouped[k]),
          9 if k[0] == "translation" else (6 if k[1] == 5 else 4),
          "partial" if len(grouped[k]) < (9 if k[0] == "translation" else (6 if k[1] == 5 else 4)) else "count covered; inspect provenance"] for k in keys])
    slack_rows = []
    for key in keys:
        losses, equivalent = [], []
        runs = sorted({d["run"] for d in deltas if d["key"] == key})
        for run_id in runs:
            block = [d for d in deltas if d["key"] == key and d["run"] == run_id]
            if block and block[0]["reversal"] and len(block) > 1:
                steady = float(np.mean([d["value"] for d in block[1:]]))
                loss = steady - block[0]["value"]
                losses.append(loss)
                if steady > 0:
                    equivalent.append(loss / steady)
        mean, std, n = stats(losses)
        slack_rows.append([label(key), "m" if key[0] == "translation" else "deg", mean, std, n,
                           stats(equivalent)[0], "steady mean minus first reversal step; equivalent loss is not a confirmed extra-command count"])
    add(["Slack", "Unit", "Mean loss", "Sample std", "Reversals with steady data", "Equivalent commands", "Definition"], slack_rows)
    repeat_rows = []
    for group, cycles in repeats.items():
        # More than one endpoint in a cycle is ambiguous, not independent repeats.
        points = np.asarray([v[0] for v in cycles.values() if len(v) == 1])
        n = len(points)
        if n >= 2:
            spread = float(np.max(np.linalg.norm(points[:, None] - points[None, :], axis=2)) * 1000)
            axis_std = (np.std(points, axis=0, ddof=1) * 1000).tolist()
        else:
            spread, axis_std = "unavailable", "unavailable"
        repeat_rows.append([group[0], n, spread, axis_std, "partial" if n < 3 else "three-cycle count covered", "only explicit repeatability.group/cycle tags; same marker/size/epoch"])
    add(["Deployed-pose repeatability", "Distinct accepted cycles", "Max pairwise spread (mm)", "Camera xyz sample std (mm)", "Coverage", "Method"],
        repeat_rows or [["unavailable", 0, "unavailable", "unavailable", "0 / 3", "No explicitly tagged accepted endpoints"]])
    chassis_rows = []
    for family, unit in (("rotation", "deg / (omega*s)"), ("translation", "mm / (velocity*s)")):
        family_samples = [s for s in chassis if s["family"] == family]
        groups = sorted({(s["command"], s["duration_s"]) for s in family_samples})
        for command, duration in groups:
            values = [s["gain"] for s in family_samples if s["command"] == command and s["duration_s"] == duration]
            mean, std, n = stats(values)
            chassis_rows.append([family, unit, command, duration, mean, std, n, "accepted common fixed-board poses only"])
        if not groups:
            chassis_rows.append([family, unit, "unavailable", "unavailable", "unavailable", "unavailable", 0,
                                 "No accepted board_pose_before/after with common board_id, metric pose and <=2 px RMS"])
    add(["Chassis pulse", "Gain unit", "Command", "Duration (s)", "Mean gain", "Sample std", "n", "Evidence"], chassis_rows)
    linear_rows = []
    for family in ("rotation", "translation"):
        values = [s for s in chassis if s["family"] == family]
        durations = {s["duration_s"] for s in values}
        if len(durations) >= 3 and len(values) >= 6:
            x = np.array([s["command"] * s["duration_s"] for s in values])
            y = np.array([s["value"] for s in values])
            design = np.column_stack((x, np.ones_like(x)))
            slope, intercept = np.linalg.lstsq(design, y, rcond=None)[0]
            residual = float(np.sqrt(np.mean((design @ np.array([slope, intercept]) - y) ** 2)))
            linear_rows.append([family, len(values), len(durations), slope, intercept, residual,
                                "descriptive affine fit only; no predefined linearity acceptance threshold"])
        else:
            linear_rows.append([family, len(values), len(durations), "unavailable", "unavailable", "unavailable",
                                "inconclusive: require at least 3 durations and 6 accepted pulses for a descriptive fit"])
    add(["Chassis duration linearity", "n", "Durations", "Affine slope", "Intercept", "Residual RMS (deg or mm)", "Decision"], linear_rows)
    transform = session.get("marker_to_wrist_transform")
    add(["Offset-dependent result", "Value", "Reason"], [
        ["Wrist positions / wrist-step constants", "unavailable", "Independent full marker-to-wrist transform unavailable" if transform is None else "Transform present but wrist reconstruction not implemented; no zero-offset assumption"],
        ["Arm-derived camera extrinsics / fit residuals", "unavailable", "No measured wrist positions; app_model_state is a prediction, not independent calibration evidence"],
        ["Pure joint rotations", "offset not required", "Requires rigid marker downstream of jogged joint; upstream attachment does not observe that joint. Low motion alone is not a valid zero-step measurement."]])
    covered = all(len(grouped[k]) >= (9 if k[0] == "translation" else (6 if k[1] == 5 else 4)) for k in keys)
    add(["Decision", "Outcome / rule"], [
        ["Protocol rule", "Adopt as-is only if EVERY step ratio is 0.85–1.15 and sample std <= 20% of mean, with planned direction/joint/sign coverage"],
        ["Current decision", "INCONCLUSIVE: " + ("criteria lack planned coverage; " if not covered else "") + "wrist-step adoption and arm-derived extrinsics unavailable; partial acquisition"],
        ["Measured constants", "unavailable; no YAML or vendor constants adopted/corrected"],
        ["Direction repeat interpretation", "Initial 3-step block plus two repeats: minimum 9 per direction; M5 6 per sign; M3/M4 4 per sign"],
        ["Zero versus unavailable", "Counts may be zero; unavailable measurements are never replaced by zero motion"]])
    criteria = []
    for key in keys:
        mean, std, n = stats(grouped[key])
        prediction = 0.04 if key[0] == "translation" else 10.0
        if n < 2:
            ratio_status = dispersion_status = "unavailable: fewer than two accepted steps"
        else:
            ratio_status = "within 0.85–1.15" if 0.85 <= mean / prediction <= 1.15 else "outside 0.85–1.15"
            dispersion_status = "within 20%" if mean > 0 and std <= 0.2 * mean else "outside 20% / zero mean"
        criteria.append([label(key), ratio_status, dispersion_status,
                         "marker-space only; wrist criterion blocked by unknown transform" if key[0] == "translation" else "pure joint orientation; wrist offset not required"])
    add(["Decision criterion", "Step ratio", "Sample dispersion", "Applicability"], criteria)
    add(["Quality rule", "Definition"], [
        ["Acceptance", "Explicit accepted tag or accepted analysis, verified rigid/planar target, known black side, marker identity, two-frame <=1 px settling and <=2 px reprojection RMS"],
        ["Joint low-motion screen", "Angles <= max(1 degree, 3x endpoint settling-rotation noise) excluded as unresolved, not measured zero; not a vendor accuracy criterion"],
        ["Repeatability tags", "repeatability: {group: identical physical reset endpoint, cycle: distinct deployment cycle}; no groups inferred from app_model_state"],
        ["Chassis tags", "board_pose_before/after: {accepted: true, board_id: same stationary floor board, rvec: Rodrigues, tvec_m: board-to-camera, reprojection_rms_px: <=2}; board z is floor normal"],
        ["Slack gaps", "Missing/rejected endpoints break contiguous runs; no bridged command gaps or inferred extra commands"]])
    add(["Excluded log line", "Timestamp", "Method", "Reason"], rejected or [["none", "unavailable", "unavailable", "No evidence rows"]])
    add(["Parse exclusion line", "Reason"], parse_errors or [["none", "No malformed/truncated lines in snapshot"]])
    output = local_path(args.output or HERE / "RESULTS.md", Path.cwd())
    output.write_text("\n\n".join(sections) + "\n")
    print(json.dumps({"report": str(output), "scope": "partial", "log_bytes": len(snapshot),
                      "parsed_records": len(records), "parse_exclusions": len(parse_errors),
                      "accepted_command_deltas": len(deltas), "excluded_evidence_rows": len(rejected),
                      "decision": "inconclusive"}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="store_true", help="Aggregate an offline JSONL snapshot into tables-only RESULTS.md")
    parser.add_argument("--log", type=Path, default=HERE / "log.jsonl")
    parser.add_argument("--session", type=Path, default=HERE / "session.json")
    parser.add_argument("--frame1", type=Path)
    parser.add_argument("--frame2", type=Path)
    parser.add_argument("--marker-id", type=int)
    parser.add_argument("--marker-side-m", type=float,
                        help="Black marker side in metres; frame-pair default 0.030, report uses per-sample metadata")
    parser.add_argument("--roi", nargs=4, type=int, default=[0, 0, 1600, 520])
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overlay", type=Path)
    parser.add_argument("--detect-only", action="store_true",
                        help="Do not apply a single metric size to mixed-size floor markers")
    args = parser.parse_args()
    if args.marker_side_m is not None and (not np.isfinite(args.marker_side_m) or args.marker_side_m <= 0):
        parser.error("Marker side must be positive and finite")
    cal = json.loads(CALIBRATION.read_text())
    if args.report:
        if args.frame1 or args.frame2 or args.detect_only or args.overlay:
            parser.error("--report cannot be combined with frame/overlay/detect-only mode")
        report(args, cal)
        return
    if args.frame1 is None:
        parser.error("--frame1 is required unless --report is selected")
    if args.marker_side_m is None:
        args.marker_side_m = 0.030
    im1, first = detect(args.frame1, cal, args.roi, args.marker_side_m, not args.detect_only)
    result = {"frame1": str(args.frame1.resolve()), "roi": args.roi,
              "marker_side_m": args.marker_side_m, "markers_first": first,
              "marker_to_wrist_offset_assumed": False,
              "intrinsics_caveat": cal.get("note", "")}
    result["metric_pose_computed"] = not args.detect_only
    if args.detect_only:
        result["marker_side_m"] = None
    vis = im1
    if args.frame2:
        vis, second = detect(args.frame2, cal, args.roi, args.marker_side_m, not args.detect_only)
        result.update({"frame2": str(args.frame2.resolve()), "markers_second": second})
        result.update(analyse_pair(first, second, args.marker_id))
    else:
        result.update({"status": "single_frame_only", "accepted": False,
                       "marker": None})
    if args.overlay:
        for entry in result.get("markers_second", first):
            pts = np.round(entry["corners_px"]).astype(np.int32)
            cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
            centre = tuple(np.round(entry["centre_px"]).astype(int))
            cv2.putText(vis, str(entry["id"]), centre,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if not cv2.imwrite(str(args.overlay), vis):
            raise ValueError("Failed to write overlay")
        result["overlay"] = str(args.overlay.resolve())
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
