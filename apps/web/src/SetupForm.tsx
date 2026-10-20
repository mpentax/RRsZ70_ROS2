import { useState } from 'react';
import { json, ApiError } from './api.ts';
import type { Setup } from './ros.ts';

type Device = {id:string;name:string;model:string};
export function SetupForm({setup,onUpdated}:{setup:Setup|null;onUpdated:()=>Promise<void>}) {
  const [email,setEmail] = useState('');
  const [region,setRegion] = useState('eu');
  const [code,setCode] = useState('');
  const [pin,setPin] = useState('');
  const [devices,setDevices] = useState<Device[]>([]);
  const [id,setId] = useState('');
  const [host,setHost] = useState('');
  const [protocol,setProtocol] = useState('L01');
  const [stage,setStage] = useState('account');
  const [busy,setBusy] = useState(false);
  const [message,setMessage] = useState('');
  async function submit(work:()=>Promise<void>) {
    setBusy(true);setMessage('');
    try { await work(); }
    catch (e) {
      if (e instanceof ApiError && e.status === 410) { setStage('account');setDevices([]);setCode('');setPin(''); }
      setMessage(e instanceof Error ? e.message : 'Setup unavailable. Check the account region, verification code, literal device IP and existing camera pattern.');
    } finally {setBusy(false);}
  }
  const configured = Boolean(setup?.device && setup?.accountConfigured && setup?.pinConfigured);
  return <section className="card setup">
    <div className="section-heading">
      <div>
        <p className="eyebrow">DEVICE &amp; CLOUD PAIRING</p>
        <h2>Account and device setup</h2>
      </div>
      <div className="button-row" style={{margin:0}}>
        <span className={`badge ${setup?.accountConfigured ? 'good' : 'warning'}`}>
          <span className="dot" />Account {setup?.accountConfigured ? 'configured' : 'required'}
        </span>
        <span className={`badge ${setup?.pinConfigured ? 'good' : 'warning'}`}>
          <span className="dot" />Camera pattern {setup?.pinConfigured ? 'saved server-side' : 'required'}
        </span>
      </div>
    </div>
    <p className="caption">
      {setup?.device ? `Selected robot: ${setup.device.name} (${setup.device.model}) at ${setup.device.host} · protocol ${setup.device.localProtocol}` : 'No selected device.'}{' '}
      Authenticate your Roborock account, select your robot and LAN IP, and save your existing camera pattern.
    </p>
    <div className="setup-grid">
      <form onSubmit={e => {e.preventDefault();void submit(async () => {await json('/api/account/code',{body:{email,region}});setStage('code');setCode('');setMessage('Code sent. Enter it here within 20 minutes.');});}}>
        <h3><span className="step">1</span>Roborock account code</h3>
        <div className="form-grid">
          <label>Account email<input type="email" placeholder="you@example.com" required autoComplete="off" value={email} onChange={e=>setEmail(e.target.value)} /></label>
          <label>Region<select value={region} onChange={e=>setRegion(e.target.value)}>{['eu','us','cn','asia'].map(r=><option key={r} value={r}>{r.toUpperCase()}</option>)}</select></label>
        </div>
        <button className={configured ? 'quiet' : 'primary'} disabled={busy}>Send sign-in / reauthentication code</button>
      </form>

      <div>
        <h3><span className="step">2</span>Verify &amp; bind robot</h3>
        {stage === 'code' && <form onSubmit={e => {e.preventDefault();const value=code;setCode('');void submit(async () => {const result=await json<{devices:Device[]}>('/api/account/login',{body:{code:value}});setDevices(result.devices);setId(result.devices[0]?.id ?? '');setEmail('');setStage('device');if (!result.devices.length) setMessage('No supported devices in this account/region. Request a new code for the correct account.');});}}>
          <label>Verification code (6 digits)<input type="password" placeholder="000000" autoComplete="off" required pattern="[0-9]{6}" value={code} onChange={e=>setCode(e.target.value)} /></label>
          <button className="primary" disabled={busy}>Verify account</button>
        </form>}
        {stage === 'device' && devices.length > 0 && <form onSubmit={e => {e.preventDefault();const value=pin;setPin('');void submit(async () => {await json('/api/account/select',{body:{id,host,localProtocol:protocol,...(value ? {pin:value}: {})}});setDevices([]);setStage('account');await onUpdated();setMessage('Device selected. ROS acquisition will report connection status.');});}}>
          <label>Device<select value={id} onChange={e=>setId(e.target.value)}>{devices.map(d=><option key={d.id} value={d.id}>{d.name} ({d.model})</option>)}</select></label>
          <div className="form-grid">
            <div>
              <label>Device IP (literal IPv4/IPv6)<input required placeholder="192.168.1.x" value={host} onChange={e=>setHost(e.target.value)} /></label>
              <p className="caption" style={{marginTop:6}}>In the Roborock app: Settings &gt; Product Info &gt; Wi-Fi Name.</p>
            </div>
            <label>Local protocol<select value={protocol} onChange={e=>setProtocol(e.target.value)}><option>L01</option><option>1.0</option></select></label>
          </div>
          <label>Existing camera pattern (digits 1–9)<input type="password" autoComplete="off" pattern="[1-9]{4,12}" value={pin} onChange={e=>setPin(e.target.value)} /></label>
          <button className="primary" disabled={busy}>Save selected device</button>
        </form>}
        {stage === 'account' && <p className="caption" style={{marginTop:14}}>Request a verification code in Step 1 when switching accounts or refreshing expired cloud camera credentials.</p>}
      </div>

      <form onSubmit={e => {e.preventDefault();const value=pin;setPin('');void submit(async () => {await json('/api/camera/pin',{body:{pin:value}});await onUpdated();setMessage('Camera pattern saved server-side. Read ROS camera status for admission errors.');});}}>
        <h3><span className="step">3</span>Camera pattern (PIN)</h3>
        <label>Update existing camera pattern<input type="password" placeholder="4–12 digits (1–9)" autoComplete="off" required pattern="[1-9]{4,12}" value={pin} onChange={e=>setPin(e.target.value)} /></label>
        <button className="quiet" disabled={busy}>Save camera pattern</button>
      </form>
    </div>
    {message && <p role="status" className="notice" style={{marginTop:16,marginBottom:0}}>{message}</p>}
  </section>;
}
