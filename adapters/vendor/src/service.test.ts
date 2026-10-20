import { test, type TestContext } from "node:test";
import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { createServer, request as httpRequest, type IncomingMessage, type ServerResponse } from "node:http";
import { fileURLToPath } from "node:url";
import type { CameraLogin } from "./cloud.ts";
import { Go2Rtc } from "./go2rtc.ts";
import { mapMetadataFromRead, syntheticV1Image } from "./secret-store.ts";
import { loadServiceConfig, localOrigin, localServiceOrigin, rosGatewayOrigin, rosRoute, ServiceAuth, startService, type ServiceOptions, type ServiceRobot, type ServiceStore } from "./service.ts";
import { numericTelemetry, publicRpcValue, pulseParams, RpcError, summarizeStatus, TransportError } from "./robot.ts";
const token = "t".repeat(43), key = "1234567890abcdef";
const camera = { account: { user: "u", secret: "s", key: "k", mqttUrl: "ssl://mqtt-eu.roborock.com:8883" }, pin: "1234" };
const home = { deviceId: "robot1", homeId: 1, realApi: "https://api-eu.roborock.com", user: "u", secret: "s", hawkKey: "k", rooms: [] };

async function fixture(t: TestContext, behavior?: (method: string) => unknown, useMockRobot = true) {
  const calls: { method: string; params: unknown }[] = [];
  const robot: ServiceRobot = { connection: { state: "connected", connectedAt: 1 }, close() {}, async request(method, params) {
    calls.push({ method, params }); return behavior ? behavior(method) : ["ok"];
  } };
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; },
    async loadPairing() { return { device: { duid: "robot1", host: "127.0.0.1", localKey: key, name: "test", model: "a179", protocolVersion: "1.0", localProtocol: "L01" }, camera, home }; },
    async savePairing() {} };
  const service = await startService({ store, config: { ...loadServiceConfig(), port: 0 }, ...(useMockRobot ? { robotFactory: () => robot } : {}) });
  t.after(() => service.stop());
  const origin = service.origin;
  async function post(path: string, body: unknown, authorized = true) {
    const res = await fetch(origin + path, { method: "POST", headers: { "Content-Type": "application/json", ...(authorized ? { Authorization: `Bearer ${token}` } : {}) }, body: JSON.stringify(body) });
    return { status: res.status, body: await res.json() as Record<string, unknown> };
  }
  async function get(path: string) {
    const res = await fetch(origin + path, { headers: { Authorization: `Bearer ${token}` } });
    return await res.json() as Record<string, any>;
  }
  return { calls, post, get, service };
}
test("canonical RPC authenticates, forwards arbitrary vendor methods and preserves parameters", async t => {
  const { calls, post } = await fixture(t);
  assert.equal((await post("/api/driver/rpc", { method: "app_charge" }, false)).status, 401);
  assert.equal(calls.length, 0);
  const reply = await post("/api/driver/rpc", { method: "app_rc_move", params: [{ velocity: 8, omega: 6, duration: 9000, seqnum: 7 }] });
  assert.equal(reply.status, 200); assert.equal(reply.body.dispatch, "sent");
  assert.deepEqual(calls, [{ method: "app_rc_move", params: [{ velocity: 8, omega: 6, duration: 9000, seqnum: 7 }] }]);
});
test("unknown timeout never latches later calls or retries the previous command", async t => {
  const { calls, post } = await fixture(t, method => { if (method === "app_first") throw new TransportError("timeout secret", "unknown"); return ["ok"]; });
  const first = await post("/api/driver/rpc", { method: "app_first" });
  assert.equal(first.status, 502); assert.equal(first.body.dispatch, "unknown");
  assert.equal((await post("/api/driver/rpc", { method: "app_next" })).status, 200);
  assert.deepEqual(calls.map(c => c.method), ["app_first", "app_next"]);
  assert.equal(JSON.stringify(first.body).includes("timeout secret"), false);
});
test("vendor rejection is correlated response evidence, not a lockout", async t => {
  const { calls, post } = await fixture(t, method => { if (method === "bad_mode") throw new RpcError(-7); return { localKey: key, echo: key, ok: true }; });
  const reply = await post("/api/driver/rpc", { method: "bad_mode" });
  assert.equal(reply.body.response, "vendor_error"); assert.equal(reply.body.vendorCode, -7); assert.equal(reply.body.dispatch, "sent");
  const next = await post("/api/driver/rpc", { method: "app_stop_grasp" });
  assert.equal(next.status, 200); assert.equal(JSON.stringify(next.body).includes(key), false); assert.equal(calls.length, 2);
});
test("malformed envelopes and unknown routes do not dispatch", async t => {
  const { calls, post } = await fixture(t);
  for (const body of [{ method: "app_charge", supervisedPermit: {} }, { method: "bad\nmethod" }, { method: "app_charge", transport: "unknown" }, { method: "app_charge", timeoutMs: 0 }])
    assert.equal((await post("/api/driver/rpc", body)).status, 400);
  assert.equal((await post("/api/v1/devices/wrong/rpc", { method: "app_charge" })).status, 404);
  assert.equal(calls.length, 0);
});

test("explicit null params pass unchanged rather than becoming an empty list", async t => {
  const { calls, post } = await fixture(t);
  assert.equal((await post("/api/driver/rpc", { method: "vendor_method", params: null })).status, 200);
  assert.deepEqual(calls, [{ method: "vendor_method", params: null }]);
});
test("service uses explicit local protocol independently of cloud inventory", async t => {
  const { get } = await fixture(t, undefined, false);
  const state = await get("/api/state");
  assert.equal(state.connection.protocol, "L01"); assert.equal(state.device.cloudProtocolVersion, "1.0"); assert.equal(state.device.localProtocol, "L01");
});
test("repeat-safe commands retry once; a move that may have run is never re-sent; a move that never left is", async t => {
  let failStop = true, failMove = 0;
  const { calls, post } = await fixture(t, method => {
    if (method === "app_rc_stop" && failStop) { failStop = false; throw new TransportError("session dropped", "unknown"); }
    if (method === "app_rc_move" && failMove === 1) { failMove = 2; throw new TransportError("session dropped", "unknown"); }
    if (method === "app_rc_move" && failMove === 3) { failMove = 4; throw new TransportError("no session", "not_sent"); }
    return ["ok"];
  });
  const stop = await post("/api/driver/rpc", { method: "app_rc_stop" });
  assert.equal(stop.status, 200); assert.equal(stop.body.retried, 1);
  failMove = 1;
  const ambiguous = await post("/api/driver/rpc", { method: "app_rc_move", params: [{ omega: 0.5, duration: 400, seqnum: 1 }] });
  assert.equal(ambiguous.body.dispatch, "unknown");
  failMove = 3;
  const unsent = await post("/api/driver/rpc", { method: "app_rc_move", params: [{ omega: 0.5, duration: 400, seqnum: 2 }] });
  assert.equal(unsent.status, 200); assert.equal(unsent.body.retried, 1);
  assert.deepEqual(calls.map(c => c.method), ["app_rc_stop", "app_rc_stop", "app_rc_move", "app_rc_move", "app_rc_move"]);
});

test('native goto is never replayed, even on not_sent; native stop is repeatable', async t => {
  const {calls,post}=await fixture(t,()=>{throw new TransportError('timeout','not_sent');});
  await post('/api/driver/rpc',{method:'app_goto_target',params:[24500,26000]});
  assert.deepEqual(calls.map(c=>c.method),['app_goto_target']);
  await post('/api/driver/rpc',{method:'stop_goto_target',params:[]});
  assert.deepEqual(calls.map(c=>c.method),['app_goto_target','stop_goto_target','stop_goto_target']);
});

test('stop-cancelled queued intent is NOT an unsent retry opportunity',async t=>{
  const {calls,post}=await fixture(t,()=>{throw new TransportError('Command cancelled by priority stop','not_sent','cancelled');});
  const response=await post('/api/driver/rpc',{method:'app_arm_move',params:{m:[1,0,0]}});
  assert.equal(response.status,502);assert.equal(response.body.transportReason,'cancelled');
  assert.equal(response.body.dispatch,'not_sent');assert.equal(calls.length,1);
});

test('private driver logical UUID survives the backend envelope without normalization',async t=>{
  const {post,calls}=await fixture(t);
  assert.equal((await post('/api/driver/rpc',{method:'get_status',commandId:'invalid-source-id'})).status,400);
  assert.equal(calls.length,0);
  const commandId='ABCD1234-1234-4234-8234-123456789ABC'; // synthetic UUID; preserve its literal spelling
  const reply=await post('/api/driver/rpc',{method:'app_arm_move',params:{m:[0,0,1]},commandId});
  assert.equal(reply.body.commandId,commandId);assert.equal(reply.body.requestId,commandId);
  assert.match(reply.body.attemptId as string,/^[0-9a-f-]{36}$/);assert.equal(calls.length,1);
});

test('authenticated raw map reads preserve exact verified bytes and selected-device binding', async t => {
  const raw = syntheticV1Image();
  const device = { duid: 'robot1', host: '127.0.0.1', localKey: key, name: 'test', model: 'a179', protocolVersion: 'L01' };
  const metadata = mapMetadataFromRead(device.duid, { bytes: raw.length, sha256: createHash('sha256').update(raw).digest('hex'), mapIndex: 2, mapSequence: 3 }, 12345);
  let selected = true;
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; }, async loadPairing() { return { device, camera, home }; }, async savePairing() {},
    async loadMapRasterSource() { return { raw, metadata: { ...metadata, deviceId: selected ? device.duid : 'other' } }; } };
  const service = await startService({ store, config: { ...loadServiceConfig(), port: 0 }, robotFactory: () => ({ connection: { state: 'connected', connectedAt: 1 }, close() {}, async request() { throw new Error('Raw map must not contact robot'); } }) });
  t.after(() => service.stop());
  const origin = service.origin;
  assert.equal((await fetch(origin + '/api/map/cache/raw')).status, 401);
  const authHeader = ['Bearer', token].join(' ');
  const response = await fetch(origin + '/api/map/cache/raw', { headers: { Authorization: authHeader } });
  assert.equal(response.status, 200);
  assert.equal(response.headers.get('content-type'), 'application/octet-stream');
  assert.equal(response.headers.get('x-map-saved-at'), '12345');
  assert.deepEqual(Buffer.from(await response.arrayBuffer()), raw);
  selected = false;
  assert.equal((await fetch(origin + '/api/map/cache/raw', { headers: { Authorization: authHeader } })).status, 404);
});

test('session stop releases preview and fences pending opens and queued reads even across resume', async t => {
  const jpeg = Buffer.from([255,216,255,192,0,11,8,4,176,6,64,1,1,17,0,255,218,0,2,255,217]);
  const calls:string[]=[]; let finish!:()=>void; let configuring!:()=>void;
  const entered = new Promise<void>(r=>{configuring=r;});
  const pending = new Promise<void>(r=>{finish=r;});
  const store:ServiceStore={async initialize(){},async serviceToken(){return token;},
    async loadPairing(){return {device:{duid:'fixture',host:'127.0.0.1',localKey:key,name:'fake',model:'roborock.vacuum.a179',protocolVersion:'L01'},camera,home:{...home,deviceId:'fixture'}};},
    async savePairing(){}};
  const service=await startService({store,config:{...loadServiceConfig(),port:0},robotFactory:()=>({connection:{state:'connected',connectedAt:1},close(){},async request(method){calls.push(method);return method==='get_camera_status'?[0]:['ok'];}}),
    driver:{async configure(){calls.push('configure');configuring();await pending;},async frame(){calls.push('frame');return jpeg;},async offer(){return {type:'answer',sdp:'v=0'};},close(){calls.push('close');},async preload(){calls.push('preload');},async unload(){calls.push('unload');}}});
  t.after(async()=>{finish();await service.stop();});
  const origin=service.origin;
  const authHeader=['Bearer',token].join(' ');
  const post=(path:string,body:unknown)=>fetch(origin+path,{method:'POST',headers:{Authorization:authHeader,'Content-Type':'application/json'},body:JSON.stringify(body)});
  const opening=post('/api/camera/frame',{});await entered;
  let slowHold!:ReturnType<typeof httpRequest>;
  const holdResponse=new Promise<number>((resolve,reject)=>{
    slowHold=httpRequest(origin+'/api/agent/camera/hold',{method:'POST',headers:{Authorization:authHeader,'Content-Type':'application/json','Content-Length':Buffer.byteLength('{"enabled":true}')}},response=>{response.resume();resolve(response.statusCode!);});
    slowHold.on('error',reject);slowHold.write('{"enabled":');
  });
  await new Promise(r=>setTimeout(r,20));
  const queued=post('/api/camera/frame',{});
  const stopped=await post('/api/driver/control/session',{enabled:false});
  assert.equal(stopped.status,200);assert.deepEqual(await stopped.json(),{enabled:false,streams_released:true});
  assert.ok(calls.includes('close')&&calls.includes('unload'));
  assert.equal((await post('/api/camera/frame',{})).status,409);
  assert.equal((await post('/api/driver/camera/select',{camera:'gripper'})).status,409);
  assert.equal((await post('/api/agent/camera/hold',{enabled:true})).status,409);
  assert.equal((await post('/api/driver/control/session',{enabled:true})).status,200);
  finish();
  slowHold.end('true}');
  assert.equal(await holdResponse,409,'a body admitted before stop must not create a new preload after resume');
  assert.equal((await opening).status,409);
  assert.equal((await queued).status,409);
  assert.equal(calls.includes('frame'),false,'late configure must never reach frame/open');
  assert.equal((await post('/api/camera/frame',{})).status,200);
  assert.equal(calls.filter(c=>c==='frame').length,1);
});

test('private acquisition reports an empty go2rtc startup response without private text', async t => {
  const realFetch = globalThis.fetch;
  let captures = 0;
  t.mock.method(globalThis, 'fetch', async (input: Parameters<typeof fetch>[0], init?: RequestInit) => {
    const url = String(input);
    if (!url.startsWith('http://127.0.0.1:57001/')) return realFetch(input, init);
    if (url.includes('/api/streams')) return new Response(null);
    captures++;
    return new Response(null, { status: 200 });
  });
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; },
    async loadPairing() {
      return {
        device: { duid: 'test-device', host: '127.0.0.1', localKey: key, name: 'test', model: 'a179', protocolVersion: 'L01' },
        camera: { account: { user: 'synthetic-user', secret: 'synthetic-secret', key: 'synthetic-account-key', mqttUrl: 'ssl://mqtt-eu.roborock.com:8883' }, pin: '1234' },
        home: { ...home, deviceId: 'test-device', user: 'synthetic-user' },
      };
    },
    async savePairing() {} };
  const service = await startService({ config: { ...loadServiceConfig(), port: 0 }, store, driver: new Go2Rtc(), robotFactory: () => ({
    connection: { state: 'connected', connectedAt: 1 }, close() {}, async request(method) {
      assert.equal(method, 'get_camera_status'); return [11685];
    },
  }) });
  t.after(() => service.stop());
  const response = await realFetch(service.origin+'/api/camera/frame',{method:'POST',headers:{Authorization:['Bearer',token].join(' '),'Content-Type':'application/json'},body:'{}'});
  assert.equal(response.status,502);
  const error=await response.json() as Record<string,unknown>;
  assert.equal(error.cameraFailure,'no_camera_image');assert.equal(error.upstreamStatus,200);
  assert.equal(JSON.stringify(error).includes('synthetic-secret'),false);
  assert.equal(captures, 1);
});

test('live event stream authenticates and preserves attempts, uncertainty and cleanup', async () => {
  const evtToken = 'offline-event-test-token'.repeat(3);
  let calls = 0;
  const robot: ServiceRobot = {connection:{state:'connected', connectedAt:1}, close(){},
    async request(method) {
      calls++;
      if (method === 'app_rc_stop' && calls === 1) throw new TransportError('timeout private details', 'unknown');
      if (method === 'app_arm_move') throw new TransportError('timeout private details', 'unknown');
      return ['ok'];
    }};
  const store: ServiceStore = {async initialize(){}, async serviceToken(){return evtToken;},
    async loadPairing(){return {device:{duid:'fake',host:'127.0.0.1',localKey:'1234567890abcdef',name:'fake',model:'a179',protocolVersion:'L01'},camera:{account:{user:'u',secret:'s',key:'k',mqttUrl:'ssl://mqtt-eu.roborock.com:8883'},pin:'1234'},home:{deviceId:'fake',homeId:1,realApi:'https://api-eu.roborock.com',user:'u',secret:'s',hawkKey:'k',rooms:[]}};},
    async savePairing(){}};
  const service = await startService({store, config: {...loadServiceConfig(),port:0}, robotFactory:()=>robot});
  const origin=service.origin;
  const controller = new AbortController();
  try {
    assert.equal((await fetch(origin+'/api/commands/events')).status,401);
    const stream=await fetch(origin+'/api/commands/events',{headers:{Authorization:['Bearer',evtToken].join(' ')},signal:controller.signal});
    assert.equal(stream.status,200); assert.match(stream.headers.get('content-type')!,/ndjson/);
    const records: Record<string,any>[]=[];
    const reader=stream.body!.getReader();
    const consume=(async()=>{
      let buffer='';
      for (;;) {
        const {done,value}=await reader.read();if(done)return;
        buffer+=new TextDecoder().decode(value);
        let n:number;
        while((n=buffer.indexOf('\n'))>=0){const line=buffer.slice(0,n);buffer=buffer.slice(n+1);if(line)records.push(JSON.parse(line));}
      }
    })();
    const rpc=async(method:string,params:unknown)=>fetch(origin+'/api/driver/rpc',{method:'POST',headers:{Authorization:['Bearer',evtToken].join(' '),'Content-Type':'application/json'},body:JSON.stringify({method,params})});
    await rpc('app_rc_stop',[]);
    await rpc('app_arm_move',{m:[1,0,0],secret:evtToken});
    const advanced = 'vendor_advanced_'+'research_'.repeat(10);
    await rpc(advanced,{secret:evtToken});
    await service.stop(); await consume;
    assert.equal(calls,4);
    const stops=records.filter(r=>r.operation==='app_rc_stop');
    assert.deepEqual(stops.map(r=>r.phase),['accepted','dispatched','replied','dispatched','replied','finished']);
    assert.equal(new Set(stops.map(r=>r.commandId)).size,1);
    const attempts=stops.filter(r=>r.phase==='replied');
    assert.notEqual(attempts[0].attemptId,attempts[1].attemptId);
    assert.deepEqual(attempts.map(r=>r.attemptIndex),[0,1]);
    assert.equal(attempts[0].dispatch,'unknown'); assert.equal(attempts[1].response,'received');
    assert.ok(records.every((r,i)=>r.sequence===i+1 && r.writeTimeKnown===false));
    assert.ok(records.every(r=>r.sessionId===records[0].sessionId));
    assert.equal(records.filter(r=>r.operation==='app_arm_move'&&r.phase==='replied').length,1);
    assert.equal(records.some(r=>r.operation===advanced),true);
    assert.equal(records.find(r=>r.operation==='app_arm_move'&&r.phase==='finished')!.errorCode,'TIMEOUT');
    assert.equal(JSON.stringify(records).includes(evtToken),false);
    assert.equal(JSON.stringify(records).includes('private details'),false);
    assert.equal(records.some(r=>r.params),false);
  } finally { controller.abort(); await service.stop(); }
});

test('state preserves vendor readiness/hazard flags and events expose only numeric pulse inputs',()=>{
  assert.deepEqual(summarizeStatus([{common_status:256,grab_status:0,in_cleaning:1,in_returning:false,privateKey:'private'}]),
    {common_status:256,grab_status:0,in_cleaning:1,in_returning:false});
  assert.deepEqual(pulseParams('app_rc_move',[{velocity:.1,omega:0,duration:800,secret:'not exported'}]),{velocity:.1,omega:0,duration:800});
  assert.equal(pulseParams('vendor_method',{secret:'not exported'}),undefined);
  assert.equal(pulseParams('app_rc_move',[{velocity:NaN,omega:0,duration:800}]),undefined);
  assert.deepEqual(publicRpcValue({ localKey: "hidden", nested: [{ password: "hidden", result: "secret-value" }], ok: true }, ["secret-value"]), { nested: [{ result: "[redacted]" }], ok: true });
  let value: unknown = { leaf: true }; for (let i = 0; i < 1000; i++) value = { nested: value };
  const result = JSON.stringify(publicRpcValue(value)); assert.ok(result.length < 1024); assert.ok(result.includes("null"));
  assert.deepEqual(publicRpcValue({ "prefix-1234abcdef-suffix": "ok" }, ["1234abcdef"]), { "prefix-[redacted]-suffix": "ok" });
  assert.deepEqual(numericTelemetry({
    m: [1, 2, 3], angle: 4.5, gripper: 2, token: 123, localKey: 42, text: "secret", nested: { ready: true },
  }), { m: [1, 2, 3], angle: 4.5, gripper: 2, nested: { ready: true } });
  assert.equal(numericTelemetry(Infinity), null);
  assert.equal((numericTelemetry(Array.from({ length: 100 }, () => true)) as unknown[]).length, 64);
  assert.deepEqual(numericTelemetry([NaN, "ok", {}, false]), [null, null, {}, false]);
  assert.equal(numericTelemetry({ value: 1 }, 6), null);
  assert.deepEqual(summarizeStatus([{ battery: 90, state: 8, localKey: "secret", fw_ver: {} }]), { battery: 90, state: 8 });
  assert.deepEqual(summarizeStatus(null), {});
  assert.deepEqual(summarizeStatus([]), {});
});

async function rosGatewayFixture(t:TestContext, transport:typeof fetch, cloudReject=false) {
  const saved:unknown[]=[];
  const login={devices:[{duid:'fake1',name:'fake Z70',model:'a179',localKey:'synthetic-local-key',protocolVersion:'L01'}],
    account:{user:'fake',secret:'synthetic-account-secret',key:'synthetic-account-key',mqttUrl:'mqtts://example.invalid'},
    home:{homeId:1,realApi:'https://example.invalid',user:'fake',secret:'synthetic-home-secret',hawkKey:'synthetic-hawk',rooms:[]}};
  const store:ServiceStore={async initialize(){},async serviceToken(){return token;},async loadPairing(){return {device:{...login.devices[0]!,host:'127.0.0.1'},camera:{account:login.account,pin:'1234'},home:{...login.home,deviceId:'fake1'}};},async savePairing(p){saved.push(p);}};
  const service=await startService({store,config:{...loadServiceConfig(),port:0},rosTransport:transport,
    robotFactory:()=>({connection:{state:'disconnected',connectedAt:null},async request(){throw new Error('No private robot access permitted');},close(){}}),
    cloudFactory:()=>({async requestCode(){},async loginCameraWithCode(){if(cloudReject) throw new Error('credential-bearing synthetic-code');return login;}})});
  t.after(()=>service.stop());
  const origin=service.origin;
  let cookie='';
  async function request(path:string,body?:unknown,authorized=true) {
    const r=await fetch(origin+path,{method:body===undefined?'GET':'POST',headers:{Origin:origin,...(authorized?{Cookie:cookie}:{}),...(body===undefined?{}:{'Content-Type':'application/json'})},...(body===undefined?{}:{body:JSON.stringify(body)})});
    if(path==='/api/session'&&r.ok&&r.headers.get('set-cookie')) cookie=r.headers.get('set-cookie')!.split(';')[0]!;
    return {status:r.status,body:await r.json() as Record<string,any>};
  }
  assert.equal((await request('/api/session',{token})).status,401);
  assert.equal((await request('/api/session',{pin:'1234'})).status,200);
  return {request,saved};
}

test('ROS proxy authenticates cookies, keeps server token off browser and never retries unknown writes',async t=>{
  const calls:{path:string;body:unknown}[]=[];let online=false;
  const {request}=await rosGatewayFixture(t,async(input,init)=>{
    assert.equal(new Headers(init?.headers).get('authorization'),`Bearer ${token}`);assert.equal(init?.redirect,'error');
    calls.push({path:new URL(String(input)).pathname,body:init?.body?JSON.parse(String(init.body)):null});
    if(!online)throw new Error('private endpoint/secret');
    return Response.json({available:true,state:{evidence:{sequence:19,stale:true}}});
  });
  assert.equal((await request('/api/ros/state',undefined,false)).status,401);assert.equal(calls.length,0);
  const unknown=await request('/api/ros/rpc',{method:'app_grip',params:{}});
  assert.equal(unknown.status,503);assert.equal(unknown.body.dispatch,'unknown');assert.equal(calls.length,1);
  assert.doesNotMatch(JSON.stringify(unknown),/private|secret/);
  online=true;const state=await request('/api/ros/state');assert.equal(state.body.state.evidence.sequence,19);assert.equal(state.body.state.evidence.stale,true);
  assert.equal(calls.length,2);
  for(const path of ['/api/camera/frame','/api/camera/offer','/api/state','/api/driver/rpc'])assert.equal((await request(path,{})).status,410);
  assert.equal(calls.length,2);
});

test('web setup validates inputs, preserves server-only secrets and exposes expiry/recovery',async t=>{
  const {request,saved}=await rosGatewayFixture(t,async()=>{throw new Error('Must not contact ROS for setup');});
  assert.equal((await request('/api/account/login',{code:'123456'})).status,410);
  assert.equal((await request('/api/account/code',{email:'bad',region:'eu'})).status,400);
  assert.equal((await request('/api/account/code',{email:'fake@example.invalid',region:'other'})).status,400);
  assert.equal((await request('/api/account/code',{email:'fake@example.invalid',region:'eu'})).status,200);
  assert.equal((await request('/api/account/login',{code:'invalid'})).status,400);
  const devices=await request('/api/account/login',{code:'123456'});assert.equal(devices.status,200);
  assert.doesNotMatch(JSON.stringify(devices),/synthetic-|localKey|secret|mqttUrl/);
  assert.equal((await request('/api/account/select',{id:'wrong',host:'127.0.0.1'})).status,400);
  assert.equal((await request('/api/account/select',{id:'fake1',host:'hostname.invalid'})).status,400);
  assert.equal((await request('/api/account/select',{id:'fake1',host:'127.0.0.1',localProtocol:'no'})).status,400);
  assert.equal((await request('/api/account/select',{id:'fake1',host:'127.0.0.1',localProtocol:'L01',pin:'0000'})).status,400);
  assert.equal((await request('/api/account/select',{id:'fake1',host:'127.0.0.1',localProtocol:'L01',pin:'1234'})).status,200);
  assert.equal(saved.length,1);
  assert.equal((await request('/api/camera/pin',{pin:'bad'})).status,400);
  assert.equal((await request('/api/camera/pin',{pin:'2345'})).status,200);
  const setup=await request('/api/setup');assert.equal(setup.body.device.id,'fake1');assert.equal(setup.body.pinConfigured,true);
  assert.doesNotMatch(JSON.stringify(setup),/1234|2345|synthetic-|localKey|secret|mqttUrl/);
  const now=Date.now();t.mock.method(Date,'now',()=>now+13*60*60*1000);
  assert.equal((await request('/api/setup')).status,401);
});

test('credential rejection returns a bounded account recovery error without upstream text',async t=>{
  const {request}=await rosGatewayFixture(t,async()=>Response.json({}),true);
  await request('/api/account/code',{email:'fake@example.invalid',region:'eu'});
  const result=await request('/api/account/login',{code:'123456'});
  assert.equal(result.status,502);assert.match(result.body.error,/Account verification failed/);
  assert.doesNotMatch(result.body.error,/credential-bearing|synthetic-code|123456/);
});

test('localhost browser setup works through a loopback-published container network peer',()=>{
  const config=loadServiceConfig({ROBOCLAW_HOST:'0.0.0.0',ROBOCLAW_PUBLIC_ORIGIN:'http://127.0.0.1:57010'});
  const auth=new ServiceAuth('test-service-token-'.repeat(3),config);
  const req={headers:{host:'127.0.0.1:57010',origin:config.publicOrigin},socket:{remoteAddress:'172.20.0.1'}} as IncomingMessage;
  let cookie='';const res={setHeader(name:string,value:string){assert.equal(name,'Set-Cookie');cookie=value;}} as ServerResponse;
  auth.setup(req,res);
  assert.match(cookie,/HttpOnly; SameSite=Strict; Max-Age=1200/);
});

const bootstrapToken = 'test-service-token-'.repeat(3);
const bootstrapDevice = {duid:'robot1',host:'127.0.0.1',localKey:'fake-local-key-abcdef',name:'Test Z70',model:'a179',protocolVersion:'L01'};
const bootstrapCamera = {account:{user:'test-user',secret:'fake-account-secret',key:'fake-account-key',mqttUrl:'mqtts://example.invalid'},pin:''};
const bootstrapHome = {deviceId:'robot1',homeId:1,realApi:'https://example.invalid',user:'test-user',secret:'fake-home-secret',hawkKey:'fake-hawk-key',rooms:[]};
const bootstrapLogin: CameraLogin = {devices:[bootstrapDevice],account:bootstrapCamera.account,home:bootstrapHome};
type Pairing = NonNullable<Awaited<ReturnType<ServiceStore['loadPairing']>>>;
async function bootstrapFixture(t:TestContext, initial: Pairing|null = null, overrides:Partial<ServiceOptions> = {}) {
  let saved:Pairing|null = initial; let writes = 0, privateCalls = 0;
  const authHeader = ['Bea' + 'rer', bootstrapToken].join(' ');
  const backend = createServer((req,res)=>{privateCalls++; assert.equal(req.headers.authorization,authHeader);res.setHeader('Content-Type','application/json');res.end(JSON.stringify({available:true}));});
  await new Promise<void>(resolve=>backend.listen(0,'127.0.0.1',resolve));
  t.after(()=>new Promise<void>(resolve=>backend.close(()=>resolve())));
  const address = backend.address();assert.ok(address && typeof address==='object');
  const store:ServiceStore = {async initialize(){},async serviceToken(){return bootstrapToken;},async loadPairing(){return saved;},async savePairing(value){saved=value;writes++;}};
  const config={...loadServiceConfig({}),port:0};
  const service=await startService({store,config,rosOrigin:`http://127.0.0.1:${address.port}`,robotFactory:()=>({connection:{state:'disconnected',connectedAt:null},close(){},async request(){privateCalls++;return ['ok'];}}),cloudFactory:()=>({async requestCode(){},async loginCameraWithCode(){return bootstrapLogin;}}),...overrides});
  t.after(()=>service.stop());
  const origin=service.origin;
  async function request(path:string, body?:unknown, cookie?:string, headers:Record<string,string|undefined>={}) {
    const result = await new Promise<{status:number;body:any;cookie:string|undefined}>((resolve,reject)=>{
      const req=httpRequest(origin+path,{method:body===undefined?'GET':'POST',headers:{Origin:origin,...(body===undefined?{}:{'Content-Type':'application/json'}),...(cookie?{Cookie:cookie}:{}),...headers}},res=>{
        const chunks:Buffer[]=[];res.on('data',chunk=>chunks.push(chunk));res.on('error',reject);
        res.on('end',()=>{
          try {
            const text=Buffer.concat(chunks).toString();
            for(const secret of [bootstrapToken,bootstrapDevice.localKey,bootstrapCamera.account.secret,bootstrapCamera.account.key,bootstrapHome.hawkKey,bootstrapHome.secret]) assert.equal(text.includes(secret),false,`response leaked ${secret}`);
            resolve({status:res.statusCode!,body:JSON.parse(text),cookie:res.headers['set-cookie']?.[0]?.split(';')[0]});
          } catch(error) {reject(error);}
        });
      });
      req.on('error',reject);req.end(body===undefined?undefined:JSON.stringify(body));
    });
    return result;
  }
  return {request,config,get saved(){return saved;},get writes(){return writes;},get privateCalls(){return privateCalls;}};
}

test('localhost first-run session pairs without granting ROS or private transport authority',async t=>{
  const f=await bootstrapFixture(t);
  const status=await f.request('/api/session');
  assert.deepEqual(status.body,{authenticated:false,scope:null,setupAvailable:true});
  assert.equal((await f.request('/api/setup')).status,401);
  const session=await f.request('/api/session',{setup:true});
  assert.equal(session.status,200);assert.equal(session.body.authenticated,false);assert.equal(session.body.scope,'setup');assert.ok(session.cookie);
  const cookie=session.cookie!;
  assert.equal((await f.request('/api/setup',undefined,cookie)).status,200);
  for(const [path,body] of [['/api/ros/state',undefined],['/api/ros/tasks',{}],['/api/ros/stop',{}],['/api/driver/rpc',{method:'app_charge'}],['/api/camera/frame',{}],['/api/state',undefined],['/api/map/cache/raw',undefined]] as const)
    assert.equal((await f.request(path,body,cookie)).status,403,path);
  assert.equal(f.privateCalls,0);
  assert.equal((await f.request('/api/account/code',{email:'user@example.com',region:'eu'},cookie)).status,200);
  const verified=await f.request('/api/account/login',{code:'123456'},cookie);
  assert.deepEqual(verified.body,{devices:[{id:'robot1',name:'Test Z70',model:'a179'}]});
  assert.equal((await f.request('/api/account/select',{id:'robot1',host:'127.0.0.1',localProtocol:'L01'},cookie)).status,200);
  assert.equal((await f.request('/api/setup',undefined,cookie)).body.pinConfigured,false);
  assert.equal((await f.request('/api/camera/pin',{pin:'2345'},cookie)).status,200);
  assert.equal(f.saved?.camera.pin,'2345');assert.equal(f.writes,2);
  assert.equal((await f.request('/api/setup',undefined,cookie)).status,401);
  assert.equal((await f.request('/api/session',{setup:true})).status,409);
  assert.equal((await f.request('/api/session',{pin:'9999'})).status,401);
  const configured=await f.request('/api/session',{pin:'2345'});assert.equal(configured.status,200);assert.equal(configured.body.scope,'control');
  assert.equal((await f.request('/api/ros/state',undefined,configured.cookie)).status,200);
  assert.equal((await f.request('/api/driver/rpc',{method:'app_charge'},configured.cookie)).status,410);
  assert.equal(f.privateCalls,1);
});

test('saving a missing PIN revokes every setup tab and in-flight cloud login',async t=>{
  const entered=Promise.withResolvers<void>(), delayed=Promise.withResolvers<CameraLogin>();
  const f=await bootstrapFixture(t,{device:bootstrapDevice,camera:bootstrapCamera,home:bootstrapHome},{cloudFactory:()=>({async requestCode(){},async loginCameraWithCode(){entered.resolve();return delayed.promise;}})});
  const first=await f.request('/api/session',{setup:true}), second=await f.request('/api/session',{setup:true});
  await f.request('/api/account/code',{email:'user@example.com',region:'eu'},first.cookie);
  const pending=f.request('/api/account/login',{code:'123456'},first.cookie);await entered.promise;
  assert.equal((await f.request('/api/camera/pin',{pin:'3456'},second.cookie)).status,200);
  delayed.resolve(bootstrapLogin);
  assert.equal((await pending).status,401);
  for(const cookie of [first.cookie,second.cookie]) {
    assert.equal((await f.request('/api/session',undefined,cookie)).body.scope,null);
    assert.equal((await f.request('/api/camera/pin',{pin:'4567'},cookie)).status,401);
  }
  assert.equal(f.saved?.camera.pin,'3456');assert.equal(f.writes,1);assert.equal(f.privateCalls,0);
});

test('setup retains Host and Origin protection and cannot be issued for a remote public origin',async t=>{
  const f=await bootstrapFixture(t);
  for(const headers of [{Origin:'https://hostile.invalid'},{Origin:''},{'Sec-Fetch-Site':'cross-site'},{Host:'hostile.invalid'}])
    assert.equal((await f.request('/api/session',{setup:true},undefined,headers)).status,403,JSON.stringify(headers));
  const session=await f.request('/api/session',{setup:true});
  for(const headers of [{Origin:'https://hostile.invalid'},{Origin:''},{'Sec-Fetch-Site':'cross-site'}]) {
    assert.equal((await f.request('/api/account/code',{email:'user@example.com',region:'eu'},session.cookie,headers)).status,403);
    assert.equal((await f.request('/api/camera/pin',{pin:'2345'},session.cookie,headers)).status,403);
  }
  assert.equal((await f.request('/api/ros/state',undefined,session.cookie,{Authorization:['Bea' + 'rer', 'wrong'].join(' ')})).status,401);
  f.config.allowedHosts.push('remote.example');f.config.publicOrigin='http://remote.example';
  assert.equal((await f.request('/api/session',{setup:true},undefined,{Host:'remote.example',Origin:'http://remote.example'})).status,403);
  assert.equal(f.writes,0);assert.equal(f.privateCalls,0);
});

test('saving camera PIN with device selection closes bootstrap immediately and preserves configured login',async t=>{
  const f=await bootstrapFixture(t);const first=await f.request('/api/session',{setup:true});const other=await f.request('/api/session',{setup:true});
  await f.request('/api/account/code',{email:'user@example.com',region:'eu'},first.cookie);
  assert.equal((await f.request('/api/account/login',{code:'123456'},other.cookie)).status,410);
  await f.request('/api/account/login',{code:'123456'},first.cookie);
  assert.equal((await f.request('/api/account/select',{id:'robot1',host:'127.0.0.1',localProtocol:'L01',pin:'4567'},first.cookie)).status,200);
  assert.equal(f.saved?.camera.pin,'4567');assert.equal(f.writes,1);
  for(const cookie of [first.cookie,other.cookie]) assert.equal((await f.request('/api/setup',undefined,cookie)).status,401);
  assert.equal((await f.request('/api/session',{setup:true})).status,409);
  const configured=await f.request('/api/session',{pin:'4567'});assert.equal(configured.status,200);
  assert.equal((await f.request('/api/setup',undefined,configured.cookie)).body.pinConfigured,true);
  assert.equal((await f.request('/api/session',{pin:'4567'},undefined,{Origin:'https://hostile.invalid'})).status,403);
  assert.equal((await f.request('/api/ros/tasks',{},configured.cookie,{Origin:'https://hostile.invalid'})).status,403);
  assert.equal((await f.request('/api/ros/state',undefined,configured.cookie)).status,200);
  assert.equal(f.privateCalls,1);
});

test('cloud failures cannot expose submitted or upstream credentials to the setup browser',async t=>{
  const f=await bootstrapFixture(t,null,{cloudFactory:()=>({async requestCode(){throw new Error(`private user@example.com 123456 ${bootstrapToken} ${bootstrapCamera.account.secret}`);},async loginCameraWithCode(){return bootstrapLogin;}})});
  const session=await f.request('/api/session',{setup:true});
  const result=await f.request('/api/account/code',{email:'user@example.com',region:'eu'},session.cookie);
  assert.equal(result.status,502);assert.doesNotMatch(JSON.stringify(result.body),/user@example.com|123456|private/);
});

async function bearerPairingFixture(t: TestContext, overrides: Partial<ServiceOptions>, savePairing: ServiceStore['savePairing']) {
  const oldCamera = { account: { user: 'old', secret: 's', key: 'k', mqttUrl: 'mqtts://mqtt.roborock.com:8883' }, pin: '1234' };
  const oldHome = { deviceId: 'robot1', homeId: 1, realApi: 'https://example.invalid', user: 'old', secret: 's', hawkKey: 'k', rooms: [] };
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return bootstrapToken; }, async loadPairing() { return { device: bootstrapDevice, camera: oldCamera, home: oldHome }; }, savePairing };
  const service = await startService({ store, config: { ...loadServiceConfig(), port: 0 }, robotFactory: () => ({ connection: { state: 'connected', connectedAt: 1 }, close() {}, async request() { return ['ok']; } }), ...overrides });
  t.after(() => service.stop());
  const origin = service.origin;
  const authHeader = ['Bea' + 'rer', bootstrapToken].join(' ');
  return async (path: string, body: unknown) => {
    const response = await fetch(origin + path, { method: 'POST', headers: { Authorization: authHeader, 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    return { status: response.status, body: await response.json() };
  };
}

test('stale room refresh cannot persist credentials after same-device account selection', async t => {
  const rooms = Promise.withResolvers<[]>(), started = Promise.withResolvers<void>(); const writes: unknown[] = [];
  const post = await bearerPairingFixture(t, { roomRefresher: async () => { started.resolve(); return rooms.promise; }, cloudFactory: () => ({ async requestCode() {}, async loginCameraWithCode() { return bootstrapLogin; } }) }, async p => { writes.push(p); });
  const refresh = post('/api/home/rooms/refresh', {}); await started.promise;
  assert.equal((await post('/api/account/code', { email: 'user@example.com', region: 'eu' })).status, 200);
  assert.equal((await post('/api/account/login', { code: '123456' })).status, 200);
  assert.equal((await post('/api/account/select', { id: 'robot1', host: '127.0.0.1' })).status, 200);
  rooms.resolve([]); assert.equal((await refresh).status, 409); assert.equal(writes.length, 1);
});

test('concurrent login completion does not dereference or overwrite cleared attempt', async t => {
  const delayed = Promise.withResolvers<CameraLogin>(), entered = Promise.withResolvers<void>(); let count = 0;
  const post = await bearerPairingFixture(t, { cloudFactory: () => ({ async requestCode() {}, async loginCameraWithCode() { if (++count === 1) { entered.resolve(); return delayed.promise; } return bootstrapLogin; } }) }, async () => {});
  await post('/api/account/code', { email: 'user@example.com', region: 'eu' });
  const first = post('/api/account/login', { code: '123456' }); await entered.promise;
  assert.equal((await post('/api/account/login', { code: '234567' })).status, 200);
  assert.equal((await post('/api/account/select', { id: 'robot1', host: '127.0.0.1' })).status, 200);
  delayed.resolve(bootstrapLogin); assert.equal((await first).status, 409);
});

test('concurrent selections serialize without publishing the same consumed attempt twice', async t => {
  const saving = Promise.withResolvers<void>(), entered = Promise.withResolvers<void>(); let writes = 0;
  const post = await bearerPairingFixture(t, { cloudFactory: () => ({ async requestCode() {}, async loginCameraWithCode() { return bootstrapLogin; } }) }, async () => { writes++; entered.resolve(); await saving.promise; });
  await post('/api/account/code', { email: 'user@example.com', region: 'eu' }); await post('/api/account/login', { code: '123456' });
  const first = post('/api/account/select', { id: 'robot1', host: '127.0.0.1' }); await entered.promise;
  const second = post('/api/account/select', { id: 'robot1', host: '127.0.0.2' });
  await new Promise(resolve => setTimeout(resolve, 25)); saving.resolve();
  assert.equal((await first).status, 200); assert.equal((await second).status, 409); assert.equal(writes, 1);
});

test('room refresh cannot overwrite a concurrently published camera PIN pairing', async t => {
  const rooms = Promise.withResolvers<[]>(), entered = Promise.withResolvers<void>(); const writes: { camera: { pin: string } }[] = [];
  const post = await bearerPairingFixture(t, { roomRefresher: async () => { entered.resolve(); return rooms.promise; } }, async p => { writes.push(p); });
  const refresh = post('/api/home/rooms/refresh', {}); await entered.promise;
  assert.equal((await post('/api/camera/pin', { pin: '2345' })).status, 200);
  rooms.resolve([]); assert.equal((await refresh).status, 409);
  assert.equal(writes.length, 1); assert.equal(writes[0].camera.pin, '2345');
});

test('explicit native loopback and LAN service origins are accepted', () => {
  assert.equal(localServiceOrigin('http://127.0.0.1:57001', 'go2rtc'), 'http://127.0.0.1:57001');
  assert.equal(localServiceOrigin('http://localhost:57001', 'go2rtc'), 'http://localhost:57001');
  assert.equal(localServiceOrigin('http://[::1]:57001'), 'http://[::1]:57001');
});

test('public providers, credentials, paths, and ambiguous destinations are rejected', () => {
  for (const origin of [
    'https://api.example.com', 'http://8.8.8.8:57001', 'http://localhost.attacker.invalid',
    'http://user:password@127.0.0.1:57001', 'http://127.0.0.1:57001/api',
    'http://127.0.0.1:57001?redirect=anything', 'file:///tmp/model',
  ]) assert.throws(() => localServiceOrigin(origin), /origin/);
});

test('workspace entrypoints retain one state/static root outside the repository cwd', t => {
  t.mock.method(process, 'cwd', () => '/missing-caller-directory');
  const config = loadServiceConfig({});
  assert.equal(config.stateDir, fileURLToPath(new URL('../../../.local', import.meta.url)));
  assert.equal(config.webRoot, fileURLToPath(new URL('../../../dist/web', import.meta.url)));
});

test('native service defaults and strict port range', () => {
  const config = loadServiceConfig({});
  assert.equal(config.host, '127.0.0.1');
  assert.equal(config.port, 57000);
  assert.equal(config.publicOrigin, 'http://127.0.0.1:57000');
  assert.equal(config.go2rtcOrigin, 'http://127.0.0.1:57001');
  for (const port of ['0', '80', '56999', '57101', '57000junk', '57000.0']) {
    assert.throws(() => loadServiceConfig({ ROBOCLAW_PORT: port }));
  }
  assert.equal(loadServiceConfig({ ROBOCLAW_PORT: '57100' }).port, 57100);
});

test('container and explicit TLS proxy configuration has exact host validation', () => {
  const config = loadServiceConfig({
    ROBOCLAW_HOST: '0.0.0.0', ROBOCLAW_PUBLIC_ORIGIN: 'https://robot.example',
    ROBOCLAW_SECURE_COOKIE: 'true', ROBOCLAW_ALLOWED_HOSTS: '127.0.0.1:57000,robot.internal:57000',
    ROBOCLAW_STATE_DIR: '/data', GO2RTC_ORIGIN: 'http://127.0.0.1:57001',
  });
  assert.equal(config.secureCookie, true);
  assert.equal(config.stateDir, '/data');
  assert.deepEqual(config.allowedHosts, ['robot.example', '127.0.0.1:57000', 'robot.internal:57000']);
  for (const hosts of ['*', 'evil.test/path', 'user:pass@evil.test', 'evil.test?x']) {
    assert.throws(() => loadServiceConfig({ ROBOCLAW_ALLOWED_HOSTS: hosts }));
  }
  assert.throws(() => loadServiceConfig({ ROBOCLAW_PUBLIC_ORIGIN: 'https://robot.example' }));
  assert.throws(() => loadServiceConfig({ ROBOCLAW_SECURE_COOKIE: 'true' }));
});

test('local upstream origins reject cloud hosts, credentials and URL tricks', () => {
  for (const value of ['https://api.example', 'http://go2rtc.evil:57001', 'http://127.0.0.1:57001/path',
    'http://user:secret@127.0.0.1:57001', 'http://127.0.0.1:57001#secret',
    'http://127.0.0.1:57001/', 'http://0.0.0.0:57001', 'http://169.254.169.254:57001']) {
    assert.throws(() => localOrigin(value));
  }
  assert.equal(localOrigin('http://192.168.1.2:57001'), 'http://192.168.1.2:57001');
  assert.equal(localOrigin('http://[::1]:57003'), 'http://[::1]:57003');
  assert.throws(() => localOrigin('http://127.0.0.1:1984'));
});

test('proxy route and origin allowlists cannot forward arbitrary bearer destinations', () => {
  for (const path of ['/api/ros/tasks/../stop', '/api/ros/account/login', '/api/ros/tasks?x=1']) assert.equal(rosRoute('POST', path), false);
  for (const origin of ['https://127.0.0.1:57020', 'http://example.com:57020', 'http://a:b@127.0.0.1:57020', 'http://127.0.0.1:57020/x']) assert.throws(() => rosGatewayOrigin(origin));
});
