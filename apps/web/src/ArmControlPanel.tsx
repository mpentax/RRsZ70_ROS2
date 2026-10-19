import { startTask } from './ros.ts';
import { robotRpc } from './rpc.ts';

export interface ArmControlPanelProps {
  busy: boolean;
  controlStopped: boolean;
  color: string;
  targetM3: string;
  targetM4: string;
  targetM5: string;
  setTargetM3: (value: string) => void;
  setTargetM4: (value: string) => void;
  setTargetM5: (value: string) => void;
  grabModeCode: number | null;
  grabModeLabel: string;
  grabActionLabel: string;
  holdToJog: boolean;
  setHoldToJog: (value: boolean) => void;
  startHoldJog: (command: { method: string; params: Record<string, unknown> }) => void;
  stopHoldJog: (sendStop?: boolean) => void;
  run: (task: () => Promise<void>) => Promise<void>;
  setOutcome: (outcome: unknown) => void;
}

export function ArmControlPanel({
  busy, controlStopped, color, targetM3, targetM4, targetM5,
  setTargetM3, setTargetM4, setTargetM5, grabModeCode, grabModeLabel, grabActionLabel,
  holdToJog, setHoldToJog, startHoldJog, stopHoldJog, run, setOutcome,
}: ArmControlPanelProps) {
  return <>
    <div className="task-panel-box">
      <h3>Curated Manipulation Stages</h3>
      <div className="button-row" style={{margin: '10px 0 0'}}>
        <button disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'deploy_arm',goal:{settle:true}})); })}>Deploy Arm</button>
        <button disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'align_target',goal:{color}})); })}>Align to Target</button>
        <button disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'move_arm',goal:{m3_deg:190,m4_deg:65,m5_deg:0}})); })}>Move Arm (Hover 190/65)</button>
        <button disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'grasp_and_lift',goal:{lift_steps:2,verify_color:color}})); })}>Grasp &amp; Lift</button>
        <button disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'release_and_stow',goal:{put_down:true}})); })}>Release &amp; Stow</button>
      </div>
      <div className="angle-bar">
        <label>M3°<input type="number" min={90} max={270} step={10} value={targetM3} onChange={e => setTargetM3(e.target.value)} /></label>
        <label>M4°<input type="number" min={35} max={170} step={10} value={targetM4} onChange={e => setTargetM4(e.target.value)} /></label>
        <label>M5°<input type="number" min={0} max={155} step={10} value={targetM5} onChange={e => setTargetM5(e.target.value)} /></label>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => {
          setOutcome(await startTask({kind:'move_arm',goal:{m3_deg:Number(targetM3),m4_deg:Number(targetM4),m5_deg:Number(targetM5)}}));
        })}>Move to angles</button>
      </div>
    </div>

    <div className="task-panel-box">
      <h3>Manual Mode Lifecycle &amp; Gripper (<code>/z70/arm/operation</code>)</h3>
      <p className="caption" style={{marginBottom: 8}}>
        Active grab mode: <strong>{grabModeLabel}</strong> · Action: <strong>{grabActionLabel}</strong>
      </p>
      <div className="button-row" style={{margin: '6px 0 0'}}>
        <button type="button" className={grabModeCode === 1 ? 'primary' : 'quiet'} disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:6}})); })}>Enter Manual Mode (flag=1)</button>
        <button type="button" className={grabModeCode === 2 ? 'primary' : 'quiet'} disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:7}})); })}>Remote Pickup Mode (flag=2)</button>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:8}})); })}>Exit Arm Mode (flag=0)</button>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:1}})); })}>Unstow Arm</button>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:2}})); })}>Stow Arm</button>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:9}})); })}>Recover Arm</button>
      </div>
      <div className="button-row" style={{margin: '8px 0 0'}}>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:4}})); })}>Close Gripper (Grip)</button>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:5}})); })}>Release Jaws In-Place (Put Down)</button>
        <button type="button" className="quiet" disabled={busy || controlStopped} onClick={() => void run(async () => { setOutcome(await startTask({kind:'arm_operation',goal:{operation:3}})); })}>Open Gripper (Recovery)</button>
        <button type="button" className="danger" disabled={controlStopped} onClick={() => { stopHoldJog(false); void robotRpc({method:'app_stop_grasp',params:[],timeoutMs:5000}).then(setOutcome).catch(() => {}); }}>Stop Arm (app_stop_grasp)</button>
      </div>
    </div>

    <div className="task-panel-box">
      <div style={{display:'flex',justifyContent:'space-between',alignItems:'center',gap:12,flexWrap:'wrap'}}>
        <h3 style={{margin:0}}>Joint Steps, Coordinated Multi-Axis &amp; Cartesian IK Jog</h3>
        <label className="checkbox-row" style={{margin:0}}>
          <input type="checkbox" checked={holdToJog} onChange={e => { setHoldToJog(e.target.checked); if (!e.target.checked) stopHoldJog(false); }} />
          <span>Hold-to-Jog (continuous press &amp; hold, stop on release)</span>
        </label>
      </div>
      <div className="joint-grid" style={{marginTop:10}}>
        {[3,4,5].map(joint => [-1,1].map(direction => {
          const m = [0, 0, 0];
          m[joint - 3] = direction === 1 ? 1 : 2;
          return (
            <button
              key={`${joint}/${direction}`}
              className="quiet"
              disabled={busy || controlStopped}
              onPointerDown={() => startHoldJog({ method: 'app_arm_move', params: { m } })}
              onPointerUp={() => stopHoldJog(true)}
              onPointerLeave={() => stopHoldJog(true)}
              onPointerCancel={() => stopHoldJog(true)}
              onClick={() => { if (!holdToJog) void run(async () => { setOutcome(await startTask({kind:'arm_step',goal:{joint,direction}})); }); }}
            >
              M{joint} {direction > 0 ? '+' : '−'} nominal step
            </button>
          );
        }))}
      </div>
      <div className="button-row" style={{margin: '10px 0 0'}}>
        {([
          ['M3+ & M4+ Down-Out', 34, 1, [1, 1, 0]],
          ['M3− & M4− Up-In', 34, -1, [2, 2, 0]],
          ['M3+ & M4− Reach-Level', 43, 1, [1, 2, 0]],
          ['M3− & M4+ Retract-Level', 43, -1, [2, 1, 0]],
        ] as const).map(([label, joint, direction, m]) => (
          <button
            key={label}
            type="button"
            className="quiet"
            disabled={busy || controlStopped}
            onPointerDown={() => startHoldJog({ method: 'app_arm_move', params: { m: [...m] } })}
            onPointerUp={() => stopHoldJog(true)}
            onPointerLeave={() => stopHoldJog(true)}
            onPointerCancel={() => stopHoldJog(true)}
            onClick={() => { if (!holdToJog) void run(async () => { setOutcome(await startTask({kind:'arm_step',goal:{joint,direction}})); }); }}
          >
            {label}
          </button>
        ))}
      </div>
      <div className="button-row" style={{margin: '10px 0 0'}}>
        {([
          [1, 'Hand Up +40mm (IK)'],
          [2, 'Hand Down −40mm (IK)'],
          [3, 'Hand Forward +40mm (IK)'],
          [4, 'Hand Back −40mm (IK)'],
        ] as const).map(([dir, label]) => (
          <button
            key={dir}
            type="button"
            className="quiet"
            disabled={busy || controlStopped}
            onPointerDown={() => startHoldJog({ method: 'app_arm_direction_move', params: { direction: dir } })}
            onPointerUp={() => stopHoldJog(true)}
            onPointerLeave={() => stopHoldJog(true)}
            onPointerCancel={() => stopHoldJog(true)}
            onClick={() => { if (!holdToJog) void run(async () => { setOutcome(await startTask({kind:'hand_step',goal:{direction:dir}})); }); }}
          >
            {label}
          </button>
        ))}
      </div>
    </div>
  </>;
}
