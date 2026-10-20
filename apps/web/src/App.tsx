import { useEffect, useRef, useState } from "react";
import { ApiError, deleteSession, json, snapshot, SESSION_EXPIRED, type SessionStatus } from "./api.ts";
import { cancelTask, listTasks, robotRpc, startTask, stopRobot, setControlSession, terminal, type Task, type TaskRequest, type RosState, type Setup } from './ros.ts';
import { SetupForm } from './SetupForm.tsx';
import { ArmControlPanel } from './ArmControlPanel.tsx';
import { MapCanvas } from './MapCanvas.tsx';
import { actionOutcome, record } from './outcomes.ts';

const COLORS = ['yellow', 'purple', 'blue', 'green', 'orange'] as const;
const ARM_STATES: Record<number, string> = { 0: 'UNKNOWN', 1: 'IN_ROBOT', 2: 'IN_PROCESS_POS', 3: 'WORKING_POS', 4: 'TRANSFER_POS' };
const GRAB_MODES: Record<number, string> = { 0: 'OFF', 1: 'MANUAL (flag=1)', 2: 'REMOTE_PICKUP (flag=2)' };
const GRAB_ACTIONS: Record<number, string> = {
  0: 'STOWED', 1: 'DEPLOYING', 2: 'STOWING', 3: 'WAITING', 4: 'GRASPING',
  5: 'PUTTING_DOWN', 7: 'CONTINUE', 8: 'MODE_CHANGING', 9: 'CONTINUE_GRAB', 10: 'CONTINUE_PUT_DOWN',
};
const ROBOT_STATES: Record<number, string> = {
  1: 'STARTING', 2: 'CHARGER_DISCONNECTED', 3: 'IDLE', 4: 'REMOTE_CONTROL_ACTIVE',
  5: 'CLEANING', 6: 'RETURNING_HOME', 7: 'MANUAL_MODE', 8: 'CHARGING',
  9: 'CHARGING_PROBLEM', 10: 'PAUSED', 11: 'SPOT_CLEANING', 12: 'ERROR',
  15: 'DOCKING', 16: 'GOING_TO_TARGET', 17: 'ZONED_CLEANING', 18: 'SEGMENT_CLEANING',
  26: 'GOING_TO_WASH_THE_MOP', 29: 'WASHING_THE_MOP', 32: 'PATROL',
  39: 'REMOTE_MANUAL', 40: 'PROGRAM_MODE', 41: 'ARM_RESETTING', 42: 'REMOTE_PICKUP', 100: 'CHARGING_COMPLETE',
};

function fmtDeg(val: unknown, transform: (deg: number) => number = d => d): string {
  return typeof val === 'number' && Number.isFinite(val) ? `${Math.round(transform(val * 180 / Math.PI))}°` : '—';
}

function urdfToVendorDeg(urdfRad: unknown[]): [string, string, string] {
  if (urdfRad.length < 3) return ['—', '—', '—'];
  const [r3, r4, r5] = urdfRad;
  return [fmtDeg(r3, d => d + 270), fmtDeg(r4, d => 180 - d), fmtDeg(r5, d => -d)];
}

export function App({ session }: { session: Promise<SessionStatus> }) {
  const [authenticated, setAuthenticated] = useState(false);
  const [pin, setPin] = useState("");
  const [state, setState] = useState<RosState | null>(null);
  const [setup, setSetup] = useState<Setup | null>(null);
  const [tasks, setTasks] = useState<Task[]>([]);
  const [color, setColor] = useState('yellow');
  const [dryRun, setDryRun] = useState(true);
  const [targetM3, setTargetM3] = useState('190');
  const [targetM4, setTargetM4] = useState('65');
  const [targetM5, setTargetM5] = useState('0');
  const [navX, setNavX] = useState('0.0');
  const [navY, setNavY] = useState('0.0');
  const [garmentCategory, setGarmentCategory] = useState('any');
  const [outcome, setOutcome] = useState<unknown>(null);
  const [mapData, setMapData] = useState<Record<string, unknown> | null>(null);
  const [error, setError] = useState("");
  const [connectionError, setConnectionError] = useState('');
  const [busy, setBusy] = useState(false);
  const [method, setMethod] = useState("get_status");
  const [params, setParams] = useState("[]");
  const [transport, setTransport] = useState<"" | "local" | "cloud">("");
  const [timeout, setTimeoutValue] = useState("12000");
  const [image, setImage] = useState("");
  const [autoStream, setAutoStream] = useState(false);
  const [sessionStopped, setSessionStopped] = useState(false);
  const [sessionBusy, setSessionBusy] = useState(false);
  const [holdToJog, setHoldToJog] = useState(false);
  const holdTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
  const holdActiveRef = useRef(false);
  const imageGeneration = useRef(0);
  const controlStopped = sessionStopped || state?.session_enabled === false;

  function stopHoldJog(sendStop = true) {
    if (holdTimerRef.current !== null) {
      clearInterval(holdTimerRef.current);
      holdTimerRef.current = null;
    }
    if (holdActiveRef.current) {
      holdActiveRef.current = false;
      if (sendStop && !controlStopped) {
        void robotRpc({ method: 'app_stop_grasp', params: [], timeoutMs: 4000 }).then(setOutcome).catch(() => {});
      }
    }
  }

  function startHoldJog(command: { method: string; params: Record<string, unknown> }) {
    if (!holdToJog || busy || controlStopped) return;
    stopHoldJog(false);
    holdActiveRef.current = true;
    const fire = () => {
      if (!holdActiveRef.current || controlStopped) return;
      void robotRpc({ ...command, ...(transport ? { transport } : {}), timeoutMs: 5000 }).then(setOutcome).catch(() => {});
    };
    fire();
    holdTimerRef.current = setInterval(fire, 650);
  }

  useEffect(() => () => stopHoldJog(false), []);

  async function controlSession(enabled: boolean) {
    setSessionBusy(true); setError('');
    if (!enabled) { setSessionStopped(true); setAutoStream(false); imageGeneration.current++; setImage(''); }
    try {
      const result = await setControlSession(enabled); setOutcome(result);
      setSessionStopped(!result.enabled);
      setState(previous => previous ? {...previous, session_enabled:result.enabled} : previous);
      if (enabled && !result.enabled) setError('Resume failed; control and streams remain stopped.');
      else if (!enabled && (!result.streams_released || result.outcomes.some(o => o.dispatch !== 1 || o.response !== 1 || o.error_code !== 'NONE')))
        setError('Session release incomplete; check outcome details.');
    } catch { setError('Session stop/resume request failed.'); }
    finally { setSessionBusy(false); }
  }

  const [setupSession, setSetupSession] = useState(false);
  const [setupAvailable, setSetupAvailable] = useState(false);
  const [accessMessage, setAccessMessage] = useState('');
  function applySession(value: SessionStatus) {
    setAuthenticated(value.authenticated);
    setSetupSession(value.scope === 'setup');
    setSetupAvailable(value.setupAvailable === true);
  }
  function clearControlState() {
    setAuthenticated(false); setState(null); setOutcome(null); setImage(""); setAutoStream(false);
  }
  async function setupUpdated() {
    if (setupSession) {
      const status = await json<SessionStatus>('/api/session');
      applySession(status);
      if (status.scope !== 'setup') {
        setSetup(null); setPin('');
        setAccessMessage('Setup complete. Sign in with your camera PIN.');
        return;
      }
    }
    setSetup(await json<Setup>('/api/setup'));
  }
  useEffect(() => {
    let active = true;
    void session.then(value => { if (active) applySession(value); }).catch(() => { if (active) setError("Session check failed."); });
    const expired = () => {
      clearControlState(); setSetupSession(false); setSetupAvailable(false); setPin(''); setSetup(null); setTasks([]);
      void json<SessionStatus>('/api/session').then(value => { if (active) applySession(value); }).catch(() => {});
    };
    window.addEventListener(SESSION_EXPIRED, expired);
    return () => { active = false; window.removeEventListener(SESSION_EXPIRED, expired); };
  }, [session]);

  // Setup must not depend on the ROS gateway being started or reachable.
  useEffect(() => {
    if (!authenticated && !setupSession) return;
    const controller = new AbortController();
    void json<Setup>('/api/setup', {signal:controller.signal}).then(value => {
      if (!controller.signal.aborted) setSetup(value);
    }).catch(e => { if (!controller.signal.aborted) setError(e instanceof Error ? e.message : 'Setup unavailable.'); });
    return () => controller.abort();
  }, [authenticated, setupSession]);

  useEffect(() => () => { if (image) URL.revokeObjectURL(image); }, [image]);

  useEffect(() => {
    if (!authenticated) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    async function refresh() {
      try {
        const [s, t] = await Promise.all([
          json<RosState>('/api/ros/state', {signal:controller.signal}),
          listTasks(controller.signal),
        ]);
        if (!controller.signal.aborted) { setState(s); setTasks(t.tasks); setConnectionError(''); }
      } catch (e) {
        if (!controller.signal.aborted) { setState(null); setConnectionError(e instanceof Error ? e.message : 'ROS unavailable.'); }
      } finally {
        if (!controller.signal.aborted) timer = setTimeout(() => void refresh(), 1500);
      }
    }
    void refresh();
    return () => { controller.abort(); clearTimeout(timer); };
  }, [authenticated]);

  useEffect(() => {
    if (!authenticated || !autoStream || controlStopped) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    async function tick() {
      try {
        const generation = imageGeneration.current;
        const frame = await snapshot(controller.signal);
        if (!controller.signal.aborted && generation === imageGeneration.current) {
          setImage(URL.createObjectURL(frame));
        }
      } catch { /* retain last frame on transient timeout */ }
      finally {
        if (!controller.signal.aborted) timer = setTimeout(() => void tick(), 2000);
      }
    }
    void tick();
    return () => { controller.abort(); clearTimeout(timer); };
  }, [authenticated, autoStream, controlStopped]);

  async function run(task: () => Promise<void>) {
    setBusy(true); setError("");
    try { await task(); } catch (e) {
      if (e instanceof ApiError && e.evidence) setOutcome(e.evidence);
      setError(e instanceof Error ? e.message : "Request failed; outcome unknown.");
    }
    finally { setBusy(false); }
  }

  function launchTask(request: TaskRequest, refreshList = false) {
    void run(async () => {
      const task = await startTask(request);
      setOutcome(task);
      if (refreshList) setTasks((await listTasks()).tasks);
    });
  }

  async function send(command: { method: string; params: unknown }) {
    setOutcome(null);
    const timeoutMs = Number(timeout);
    if (!Number.isSafeInteger(timeoutMs) || timeoutMs <= 0) throw new Error("Timeout must be a positive integer.");
    if (timeoutMs > 30000) throw new Error('Timeout must not exceed 30000 ms.');
    setOutcome(await robotRpc({...command, ...(transport ? {transport} : {}), timeoutMs}));
  }

  if (setupSession) return <main className="app-shell">
    <h1>Initial local setup</h1>
    <p className="muted">Setup only: control, camera and ROS access remain locked. Saving the camera PIN ends this setup session; then sign in normally.</p>
    {error && <p role="alert" className="error">{error}</p>}
    <SetupForm setup={setup} onUpdated={setupUpdated} />
    <button className="quiet" onClick={() => void run(async () => { await deleteSession(); setSetupSession(false); setSetup(null); })}>Exit setup</button>
  </main>;

  if (!authenticated) return <main className="access-page">
    <div className="access-shell">
      <div className="brand">
        <div className="brand-mark">R<span /></div>
        <div>
          <span className="brand-name">RoboClaw</span>
          <span className="brand-subtitle">ROBOROCK SAROS Z70 · ROS 2 CONTROL PLANE</span>
        </div>
      </div>
      <section className="card access-card">
        <p className="eyebrow">OPERATOR AUTHENTICATION</p>
        <h1>RoboClaw</h1>
        {accessMessage && <p role="status" className="notice">{accessMessage}</p>}
        {setupAvailable ? <>
          <p className="muted">No camera PIN is saved. Pair your account and device locally before signing in. No service token is needed.</p>
          <button className="primary" disabled={busy} onClick={() => void run(async () => {
            try {
              applySession(await json<SessionStatus>('/api/session', {body:{setup:true}}));
            } catch (e) {
              if (e instanceof ApiError && e.status === 409) applySession(await json<SessionStatus>('/api/session'));
              throw e;
            }
          })}>Start local setup</button>
        </> : <>
        <p className="muted">Enter your configured Roborock camera pattern to unlock the local ROS 2 control and observation console.</p>
        <form onSubmit={e => { e.preventDefault(); const value = pin; setPin(""); void run(async () => {
          const result = await json<SessionStatus>("/api/session", { body: { pin: value } }); applySession(result); setAccessMessage('');
        }); }}>
          <label>Camera PIN<input type="password" inputMode="numeric" pattern="[1-9]{4,12}" placeholder="4–12 digits (1–9)" autoComplete="off" value={pin} onChange={e => setPin(e.target.value)} /></label>
          <button className="primary" disabled={busy || !pin}>Sign in</button>
        </form>
        </>}
        {error && <p role="alert" className="error">{error}</p>}
        <div className="access-footnote">
          <span className="dot good-text" />
          <span>Camera PIN creates a same-origin session; never stored in browser storage.</span>
        </div>
      </section>
    </div>
  </main>;

  const robotState = record(state?.state);
  const cameraState = record(state?.camera);
  const armState = record(state?.arm);
  const calState = record(state?.calibration);
  const batteryPct = state?.state?.battery_known ? Math.round(state.state.battery_fraction * 100) : null;
  const selectedCam = cameraState.selected_camera === 2 ? 'gripper' : cameraState.selected_camera === 1 ? 'front' : null;
  const armValidity = record(armState.validity);
  const armJointsMsg = record(armState.joints);
  const armJointPositions = Array.isArray(armJointsMsg.position) ? armJointsMsg.position : [];
  const armPositions = Array.isArray(armState.position_rad) ? armState.position_rad : [];
  const armStateLabel = typeof armState.state === 'number'
    ? (ARM_STATES[armState.state] ?? `STATE_${armState.state}`)
    : armValidity.value === 1 ? 'DEPLOYED (DEAD-RECKONED)'
    : armValidity.value === 3 ? 'IN_ROBOT (STOWED)'
    : typeof armValidity.reason === 'string' && armValidity.reason ? armValidity.reason : 'UNAVAILABLE';
  const [vendorM3, vendorM4, vendorM5] = armJointPositions.length >= 3
    ? urdfToVendorDeg(armJointPositions)
    : [fmtDeg(armPositions[0]), fmtDeg(armPositions[1]), fmtDeg(armPositions[2])];
  const robotStateCode = typeof robotState.robot_state === 'number' ? robotState.robot_state : typeof robotState.state_code === 'number' ? robotState.state_code : null;
  const robotStateName = robotStateCode !== null ? (ROBOT_STATES[robotStateCode] ?? `CODE_${robotStateCode}`) : 'UNKNOWN';
  const commonStatus = typeof robotState.common_status === 'number' ? robotState.common_status : null;
  const grabModeCode = commonStatus !== null ? ((commonStatus >> 10) & 3) : null;
  const grabActionCode = typeof robotState.grab_status === 'number' && robotState.grab_status_known
    ? robotState.grab_status
    : commonStatus !== null ? ((commonStatus >> 6) & 15) : null;
  const grabModeLabel = grabModeCode !== null ? (GRAB_MODES[grabModeCode] ?? `MODE_${grabModeCode}`) : '—';
  const grabActionLabel = grabActionCode !== null ? (GRAB_ACTIONS[grabActionCode] ?? `ACT_${grabActionCode}`) : '—';
  const activeTasks = tasks.filter(t => !terminal(t)).length;

  return <main className="app-shell">
    <header className="app-header">
      <div className="brand">
        <div className="brand-mark">R<span /></div>
        <div>
          <span className="brand-name">RoboClaw</span>
          <span className="brand-subtitle">SAROS Z70 · ROS 2 CONTROL &amp; MANIPULATION CONSOLE</span>
        </div>
      </div>
      <div className="header-actions">
        <span className={`badge ${state?.available && state?.state?.connected ? 'good' : 'warning'}`}>
          <span className="dot" />
          {state?.available ? (state?.state?.connected ? `ONLINE · ${robotStateName}` : 'ROBOT DISCONNECTED') : 'ROS OFFLINE'}
        </span>
        {batteryPct !== null && (
          <span className={`badge ${batteryPct <= 20 ? 'warning' : 'good'}`}>
            Battery {batteryPct}%
          </span>
        )}
        <button className="stop danger-stop" onClick={() => { void stopRobot().then(setOutcome).catch(e => setError(`Stop failed: ${e instanceof Error ? e.message : 'ROS response unavailable.'}`)); }}>STOP ALL</button>
        <button className="stop danger" onClick={() => void controlSession(false)}>Stop control &amp; streams</button>
        {controlStopped && <button className="primary" disabled={sessionBusy} onClick={() => void controlSession(true)}>Resume control &amp; streams</button>}
        <button className="quiet" disabled={busy} onClick={() => void run(async () => { await deleteSession(); clearControlState(); })}>Sign out</button>
      </div>
    </header>

    <div className="page-heading">
      <div>
        <p className="eyebrow">SHARED ROS 2 AUTHORITY</p>
        <h1>RRsZ70 ROS controls</h1>
        <p className="muted">Unified ROS 2 teleoperation, manipulation, and autonomous navigation console.</p>
      </div>
      <div className="sync-status">
        <span className={`dot ${state?.available ? 'good-text' : 'warning-text'}`} />
        <div>
          <strong>{state?.available ? 'ROS driver available' : 'ROS unavailable — inspect ros2 launch/topic/action. No direct-device fallback.'}</strong>
          <small>
            {state?.state?.connected ? 'Connected' : 'Disconnected'}; battery {batteryPct !== null ? `${batteryPct}%` : 'unknown'}{state?.state?.evidence.stale ? ' (stale)' : ''}
          </small>
        </div>
      </div>
    </div>

    {controlStopped && (
      <div className="motion-hold" role="status">
        <div className="hold-icon">!</div>
        <div>
          <h2>Control session suspended</h2>
          <p>Control and camera acquisition are suspended. Click Resume to re-enable streams.</p>
        </div>
        <button className="primary" disabled={sessionBusy} onClick={() => void controlSession(true)}>Resume control &amp; streams</button>
      </div>
    )}

    {error && <div role="alert" className="error service-error"><span>{error}</span><button className="quiet" onClick={() => setError('')}>Dismiss</button></div>}
    {connectionError && <div role="alert" className="error service-error"><span>{connectionError}</span></div>}

    <section className="card" style={{marginBottom: 22}}>
      <div className="section-heading">
        <div>
          <p className="eyebrow">LIVE TELEMETRY SUMMARY</p>
          <h2>Connection and battery</h2>
        </div>
        <button className="quiet" disabled={busy} onClick={() => void run(async () => { setState(await json<RosState>("/api/ros/state")); })}>Read shared state</button>
      </div>
      <div className="battery-heading">
        <span>{state?.state?.connected ? 'Connected' : 'Disconnected'} · {robotStateName}{state?.state?.evidence.stale ? ' (stale)' : ''}</span>
        <strong>{batteryPct !== null ? `${batteryPct}%` : 'unknown'}</strong>
      </div>
      <progress className={`battery ${batteryPct !== null && batteryPct <= 20 ? 'low' : ''}`} max={100} value={batteryPct ?? 0} />

      <div className="operator-status" style={{marginTop: 20, marginBottom: 0}}>
        <div className="stat-tile">
          <span className="eyebrow">ROBOT STATUS</span>
          <strong>{robotStateName}</strong>
          <small>Error: {String(robotState.error_code ?? 0)} · Docked: {robotStateCode === 8 || robotStateCode === 100 ? 'Yes' : 'No'}</small>
        </div>
        <div className="stat-tile">
          <span className="eyebrow">ACTIVE CAMERA LENS</span>
          <strong>{selectedCam === 'gripper' ? 'GRIPPER · 1600×896' : selectedCam === 'front' ? 'FRONT · 1600×1200' : 'UNCONFIRMED'}</strong>
          <small>Bit-15 confirmed: {cameraState.selection_confirmed ? 'Yes' : 'No'}</small>
        </div>
        <div className="stat-tile">
          <span className="eyebrow">ARM ESTIMATE</span>
          <strong>{armStateLabel}</strong>
          <small>
            {(armJointPositions.length >= 3 || armPositions.length >= 3)
              ? `M3 ${vendorM3} · M4 ${vendorM4} · M5 ${vendorM5}`
              : String(armState.unknown_reason || 'Stowed / unanchored')}
            {grabModeCode !== null ? ` · Grab ${grabModeLabel} (${grabActionLabel})` : ''}
          </small>
        </div>
        <div className="stat-tile">
          <span className="eyebrow">CHASSIS CALIBRATION</span>
          <strong>{calState.ready ? 'Calibrated' : 'Uncalibrated'}</strong>
          <small>
            VO {String(calState.vo_accepted ?? 0)} ok / {String(calState.vo_rejected ?? 0)} rej · cam_x {typeof calState.active_camera_x_m === 'number' ? `${calState.active_camera_x_m.toFixed(3)}m` : '—'}
          </small>
        </div>
        <div className="stat-tile">
          <span className="eyebrow">PAIRED DEVICE</span>
          <strong>{setup?.device ? setup.device.name : 'Not configured'}</strong>
          <small>{setup?.device ? `${setup.device.host} (${setup.device.localProtocol})` : 'Configure in Setup below'}</small>
        </div>
      </div>

      <details className="subdetails">
        <summary>Raw shared ROS state JSON</summary>
        <pre className="readings">{JSON.stringify(state, null, 2)}</pre>
      </details>
    </section>

    <div className="dashboard-grid">
      <div className="camera-workspace">
        <section className="card camera-card">
          <div className="section-heading">
            <div>
              <p className="eyebrow">SHARED ROS CAMERA SAMPLE</p>
              <h2>Camera</h2>
            </div>
            <span className={`badge ${selectedCam ? 'good' : ''}`}>
              <span className="dot" />
              {selectedCam === 'gripper' ? 'GRIPPER LENS · 1600×896' : selectedCam === 'front' ? 'FRONT LENS · 1600×1200' : 'LENS UNKNOWN'}
            </span>
          </div>
          <p className="caption" style={{marginBottom: 12}}>Reads latest shared ROS sample, without opening a preview or switching lens.</p>
          <div className="button-row" style={{marginTop: 0, marginBottom: 14}}>
            <button className="primary" disabled={busy || controlStopped} onClick={() => void run(async () => {
              const generation = imageGeneration.current;
              const frame = await snapshot(new AbortController().signal);
              if (generation === imageGeneration.current) setImage(URL.createObjectURL(frame));
            })}>Capture frame</button>
            <button type="button" className={autoStream ? 'primary' : 'quiet'} disabled={controlStopped} onClick={() => setAutoStream(v => !v)}>
              {autoStream ? 'Pause live poll' : 'Live poll (2s)'}
            </button>
            {(['front','gripper'] as const).map(camera => (
              <button key={camera} className={selectedCam === camera ? 'primary' : 'quiet'} disabled={busy || controlStopped} onClick={() => void run(async () => setOutcome(await json('/api/ros/camera/select', {body:{camera},action:true,timeout:23000})))}>
                Select {camera}
              </button>
            ))}
          </div>
          <div className="video-stage" style={{marginBottom: 0}}>
            {image ? (
              <img src={image} alt="Robot camera frame" style={{ width: "100%", height: "100%", objectFit: "contain", display: "block" }} />
            ) : (
              <div className="camera-placeholder">
                <svg viewBox="0 0 24 24" aria-hidden="true">
                  <path d="M4 7h3l2-2h6l2 2h3a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V9a2 2 0 0 1 2-2Z" />
                  <circle cx="12" cy="13" r="3.5" />
                </svg>
                <strong>No snapshot loaded in this browser tab</strong>
                <span>Click “Capture frame” or enable “Live poll (2s)” above to view the active ROS camera feed.</span>
              </div>
            )}
            <div className="video-overlay">
              <span>{selectedCam ? `${selectedCam.toUpperCase()} LENS` : 'SHARED STREAM'}</span>
              {autoStream && <span>LIVE POLL · 0.5 HZ</span>}
            </div>
          </div>
        </section>

        <MapCanvas
          mapData={mapData}
          busy={busy}
          selectedTarget={{ x: Number(navX) || 0, y: Number(navY) || 0 }}
          onSelectTarget={(x, y) => { setNavX(x.toFixed(2)); setNavY(y.toFixed(2)); }}
          onReadMap={() => void run(async () => {
            const res = await json<Record<string, unknown>>('/api/ros/map');
            setMapData(res);
            setOutcome(res);
          })}
        />

        <section className="card">
          <div className="section-heading">
            <div>
              <p className="eyebrow">LAST DISPATCH RECEIPT</p>
              <h2>Latest outcome</h2>
            </div>
            {outcome !== null && <span className="badge">{actionOutcome(outcome).toUpperCase()}</span>}
          </div>
          <pre className="readings" style={{marginBottom: 0}}>{outcome ? JSON.stringify(outcome, null, 2) : 'No command or task dispatched in this session yet.'}</pre>
        </section>
      </div>

      <div className="telemetry-column">
        <section className="card">
          <div className="section-heading">
            <div>
              <p className="eyebrow">MANIPULATION &amp; MOTION</p>
              <h2>Shared ROS tasks</h2>
            </div>
            <span className={`badge ${activeTasks > 0 ? 'warning' : 'good'}`}>
              <span className="dot" />{activeTasks > 0 ? `${activeTasks} ACTIVE` : `${tasks.length} ${tasks.length === 1 ? 'TASK' : 'TASKS'}`}
            </span>
          </div>

          <form className="task-panel-box" onSubmit={e => { e.preventDefault(); launchTask({kind:'pick',goal:{color,dry_run:dryRun}}, true); }}>
            <h3>Autonomous Object Pickup (<code>/z70/pick</code>)</h3>
            <div className="chip-row">
              {COLORS.map(c => (
                <button key={c} type="button" className={`chip ${color === c ? 'active' : ''}`} onClick={() => setColor(c)}>
                  <span className={`swatch swatch-${c}`} />{c}
                </button>
              ))}
            </div>
            <div className="form-grid" style={{alignItems: 'end', marginTop: 10}}>
              <label>Target color<input required pattern="[A-Za-z0-9_-]{1,32}" value={color} onChange={e => setColor(e.target.value)} /></label>
              <label className="checkbox-row">
                <input type="checkbox" checked={dryRun} onChange={e => setDryRun(e.target.checked)} />
                <span>Observation-only dry run</span>
              </label>
            </div>
            <div className="button-row" style={{marginBottom: 0}}>
              <button className="primary" disabled={busy || controlStopped}>Start Pick</button>
              <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => launchTask({kind:'stack',goal:{base_color:'yellow',middle_color:'purple',top_color:'blue',dry_run:dryRun}}, true)}>Start 3-Ball Stack (yellow→purple→blue)</button>
            </div>
          </form>

          <div className="task-panel-box">
            <h3>Navigation &amp; Garment Patrol (<code>/z70/nav/pin_and_go</code> · <code>/z70/deliver_garment</code> · <code>/z70/patrol_collect</code>)</h3>
            <div className="form-grid" style={{alignItems: 'end', marginTop: 10}}>
              <label>Target / Drop X (m)<input type="number" step="0.05" value={navX} onChange={e => setNavX(e.target.value)} /></label>
              <label>Target / Drop Y (m)<input type="number" step="0.05" value={navY} onChange={e => setNavY(e.target.value)} /></label>
              <label>Garment category<input value={garmentCategory} onChange={e => setGarmentCategory(e.target.value)} /></label>
            </div>
            <div className="button-row" style={{marginBottom: 0}}>
              <button type="button" className="primary" disabled={busy || controlStopped} onClick={() => launchTask({ kind: 'pin_and_go', goal: { target_x_m: Number(navX), target_y_m: Number(navY), max_duration_s: 120, dry_run: dryRun } }, true)}>Pin &amp; Go</button>
              <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => launchTask({ kind: 'deliver_garment', goal: { category: garmentCategory, drop_x_m: Number(navX), drop_y_m: Number(navY), max_duration_s: 300, allow_towing: false, dry_run: dryRun } }, true)}>Deliver Garment</button>
              <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => launchTask({ kind: 'patrol_collect', goal: { drop_x_m: Number(navX), drop_y_m: Number(navY), max_duration_s: 600, max_items: 5, allow_towing: false, dry_run: dryRun } }, true)}>Patrol &amp; Collect</button>
            </div>
          </div>

          <ArmControlPanel
            busy={busy}
            controlStopped={controlStopped}
            color={color}
            targetM3={targetM3}
            targetM4={targetM4}
            targetM5={targetM5}
            setTargetM3={setTargetM3}
            setTargetM4={setTargetM4}
            setTargetM5={setTargetM5}
            grabModeCode={grabModeCode}
            grabModeLabel={grabModeLabel}
            grabActionLabel={grabActionLabel}
            holdToJog={holdToJog}
            setHoldToJog={setHoldToJog}
            startHoldJog={startHoldJog}
            stopHoldJog={stopHoldJog}
            launchTask={launchTask}
            setOutcome={setOutcome}
          />

          <div className="task-panel-box">
            <h3>Bounded Chassis Pulse Nudge (<code>/z70/base/pulse</code>)</h3>
            <div className="drive-pad" style={{margin: '10px auto 4px'}}>
              {([
                ['drive-forward', '▲ Forward', 0.1, 0],
                ['drive-left', '↺ Turn Left', 0, 0.4],
                ['drive-right', '↻ Turn Right', 0, -0.4],
                ['drive-backward', '▼ Back', -0.1, 0],
              ] as const).map(([cls, label, linear_mps, angular_radps]) => (
                <button key={cls} type="button" className={`drive-button ${cls}`} disabled={busy || controlStopped} onClick={() => launchTask({kind:'base_pulse',goal:{linear_mps,angular_radps,duration:{sec:0,nanosec:400000000}}})}>{label}</button>
              ))}
            </div>
          </div>

          <div className="subdetails">
            <h3>Task History &amp; Live Queue</h3>
            <TaskList tasks={tasks} cancel={id => { void cancelTask(id).then(setOutcome).catch(e => setError(e instanceof Error ? e.message : 'Cancellation failed.')); }} />
          </div>
        </section>

        <details className="card diagnostics">
          <summary>Advanced vendor RPC <span className="muted">Same ROS dispatcher &amp; task exclusion</span></summary>
          <p className="caption">Same ROS dispatcher, task exclusion and outcome evidence.</p>
          <div className="button-row" style={{margin: '10px 0'}}>
            {[
              ['get_status', '[]'],
              ['get_camera_status', '[]'],
              ['app_get_arm_status', '[]'],
              ['app_open_gripper', '{}'],
              ['app_stop_grasp', '[]'],
            ].map(([m, p]) => (
              <button key={m} type="button" className="chip" onClick={() => { setMethod(m!); setParams(p!); }}>{m}</button>
            ))}
          </div>
          <div className="form-grid">
            <label>Transport<select value={transport} onChange={e => setTransport(e.target.value as typeof transport)}><option value="">Service default</option><option>local</option><option>cloud</option></select></label>
            <label>Timeout (ms)<input value={timeout} onChange={e => setTimeoutValue(e.target.value)} /></label>
          </div>
          <form onSubmit={e => { e.preventDefault(); void run(() => send({ method, params: JSON.parse(params) })); }}>
            <label>Method<input required value={method} onChange={e => setMethod(e.target.value)} /></label>
            <label>Params (JSON)<textarea rows={3} value={params} onChange={e => setParams(e.target.value)} /></label>
            <button className="primary" style={{marginTop: 12}} disabled={busy || controlStopped}>Send RPC once</button>
          </form>
        </details>
      </div>
    </div>

    <SetupForm setup={setup} onUpdated={setupUpdated} />

    <footer className="app-footer">
      <span>ROBOCLAW · ROBOROCK SAROS Z70 ROS 2 CONTROL PLANE</span>
      <span>SHARED ROS 2 AUTHORITY · FOXGLOVE WS :8765 · WEB :57010</span>
    </footer>
  </main>;
}

export function TaskList({tasks,cancel}:{tasks:Task[];cancel:(id:string)=>void}) {
  if (!tasks.length) return <ul aria-label="Shared ROS tasks" className="task-list"><li className="task-empty">No ROS tasks recorded in this session yet.</li></ul>;
  return <ul aria-label="Shared ROS tasks" className="task-list">{tasks.map(task => {
    const done = terminal(task);
    const badgeClass = task.status === 'succeeded' ? 'good' : task.status === 'executing' || task.status === 'accepted' ? 'warning' : '';
    return <li key={task.id} data-task-id={task.id} className="task-item">
      <div className="task-item-header">
        <div>
          <span className={`badge ${badgeClass}`} style={{marginRight: 8}}>{task.status.toUpperCase()}</span>
          <strong>{task.kind}</strong>
          <p className="caption" style={{margin: '6px 0 0'}}>{task.id} — {task.kind}: {task.status}, phase {task.phase}</p>
        </div>
        <button className={done ? 'quiet' : 'danger'} disabled={done} onClick={() => cancel(task.id)}>Cancel {task.id}</button>
      </div>
      <pre className="readings" style={{marginBottom: 0}}>{JSON.stringify({feedback:task.feedback,result:task.result},null,2)}</pre>
    </li>;
  })}</ul>;
}
