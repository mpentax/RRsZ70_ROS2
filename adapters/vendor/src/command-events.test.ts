import { test } from 'node:test';
import assert from 'node:assert/strict';
import { startService, type ServiceRobot, type ServiceStore } from './service.ts';
import { loadServiceConfig } from './service-config.ts';
import { summarizeStatus, TransportError } from './rpc-outcome.ts';
import { pulseParams } from './command-events.ts';

// All credentials and transport here are synthetic; no disk store or robot.
test('live event stream authenticates and preserves attempts, uncertainty and cleanup', async () => {
  const token = 'offline-event-test-token'.repeat(3);
  let calls = 0;
  const robot: ServiceRobot = {connection:{state:'connected', connectedAt:1}, close(){},
    async request(method) {
      calls++;
      if (method === 'app_rc_stop' && calls === 1) throw new TransportError('timeout private details', 'unknown');
      if (method === 'app_arm_move') throw new TransportError('timeout private details', 'unknown');
      return ['ok'];
    }};
  const store: ServiceStore = {async initialize(){}, async serviceToken(){return token;},
    async loadPairing(){return {device:{duid:'fake',host:'127.0.0.1',localKey:'1234567890abcdef',name:'fake',model:'a179',protocolVersion:'L01'},camera:{account:{user:'u',secret:'s',key:'k',mqttUrl:'ssl://mqtt-eu.roborock.com:8883'},pin:'1234'},home:{deviceId:'fake',homeId:1,realApi:'https://api-eu.roborock.com',user:'u',secret:'s',hawkKey:'k',rooms:[]}};},
    async savePairing(){}};
  const service = await startService({store, config: {...loadServiceConfig(),port:0}, robotFactory:()=>robot});
  const origin=service.origin;
  const controller = new AbortController();
  try {
    assert.equal((await fetch(origin+'/api/commands/events')).status,401);
    const stream=await fetch(origin+'/api/commands/events',{headers:{Authorization:`Bearer ${token}`},signal:controller.signal});
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
    const rpc=async(method:string,params:unknown)=>fetch(origin+'/api/driver/rpc',{method:'POST',headers:{Authorization:`Bearer ${token}`,'Content-Type':'application/json'},body:JSON.stringify({method,params})});
    await rpc('app_rc_stop',[]);
    await rpc('app_arm_move',{m:[1,0,0],secret:token});
    const advanced = 'vendor_advanced_'+'research_'.repeat(10);
    await rpc(advanced,{secret:token});
    await service.stop(); await consume;
    assert.equal(calls,4); // One safe stop retry; uncertain arm move never replayed.
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
    assert.equal(JSON.stringify(records).includes(token),false);
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
});
