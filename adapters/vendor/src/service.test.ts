import { test, type TestContext } from "node:test";
import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { request as httpRequest } from "node:http";
import { Go2Rtc } from "./go2rtc.ts";
import { syntheticV1Image } from "./map-raster-fixture.ts";
import { mapMetadataFromRead } from "./private-map-cache.ts";
import { startService, type ServiceRobot, type ServiceStore } from "./service.ts";
import { loadServiceConfig } from "./service-config.ts";
import { RpcError } from "./robot.ts";
import { numericTelemetry, publicRpcValue, pulseParams, summarizeStatus, TransportError } from "./rpc-outcome.ts";
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
