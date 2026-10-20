import test from 'node:test';
import assert from 'node:assert/strict';
import { createService, type ServiceStore } from './service.ts';
import { loadServiceConfig } from './service-config.ts';
import { rosRoute, rosGatewayOrigin } from './ros-gateway.ts';

const token='z'.repeat(43), pin='1234';
async function fixture(t:import('node:test').TestContext, transport:typeof fetch, cloudReject=false) {
  const saved:unknown[]=[];
  const login={devices:[{duid:'fake1',name:'fake Z70',model:'a179',localKey:'synthetic-local-key',protocolVersion:'L01'}],
    account:{user:'fake',secret:'synthetic-account-secret',key:'synthetic-account-key',mqttUrl:'mqtts://example.invalid'},
    home:{homeId:1,realApi:'https://example.invalid',user:'fake',secret:'synthetic-home-secret',hawkKey:'synthetic-hawk',rooms:[]}};
  const store:ServiceStore={async initialize(){},async serviceToken(){return token;},async loadPairing(){return {device:{...login.devices[0]!,host:'127.0.0.1'},camera:{account:login.account,pin},home:{...login.home,deviceId:'fake1'}};},async savePairing(p){saved.push(p);}};
  const config={...loadServiceConfig(),port:0};
  const service=await createService({store,config,rosTransport:transport,
    robotFactory:()=>({connection:{state:'disconnected',connectedAt:null},async request(){throw new Error('No private robot access permitted');},close(){}}),
    cloudFactory:()=>({async requestCode(){},async loginCameraWithCode(){if(cloudReject) throw new Error('credential-bearing synthetic-code');return login;}})});
  await service.start();t.after(()=>service.stop());
  const origin=service.origin;
  let cookie='';
  async function request(path:string,body?:unknown,authorized=true) {
    const r=await fetch(origin+path,{method:body===undefined?'GET':'POST',headers:{Origin:origin,...(authorized?{Cookie:cookie}:{}),...(body===undefined?{}:{'Content-Type':'application/json'})},...(body===undefined?{}:{body:JSON.stringify(body)})});
    if(path==='/api/session'&&r.ok&&r.headers.get('set-cookie')) cookie=r.headers.get('set-cookie')!.split(';')[0]!;
    return {status:r.status,body:await r.json() as Record<string,any>};
  }
  assert.equal((await request('/api/session',{token})).status,401);
  assert.equal((await request('/api/session',{pin})).status,200);
  return {request,saved};
}

test('ROS proxy authenticates cookies, keeps server token off browser and never retries unknown writes',async t=>{
  const calls:{path:string;body:unknown}[]=[];let online=false;
  const {request}=await fixture(t,async(input,init)=>{
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
  const {request,saved}=await fixture(t,async()=>{throw new Error('Must not contact ROS for setup');});
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
  const {request}=await fixture(t,async()=>Response.json({}),true);
  await request('/api/account/code',{email:'fake@example.invalid',region:'eu'});
  const result=await request('/api/account/login',{code:'123456'});
  assert.equal(result.status,502);assert.match(result.body.error,/Account verification failed/);
  assert.doesNotMatch(result.body.error,/credential-bearing|synthetic-code|123456/);
});

test('proxy route and origin allowlists cannot forward arbitrary bearer destinations',()=>{
  for(const path of ['/api/ros/tasks/../stop','/api/ros/account/login','/api/ros/tasks?x=1'])assert.equal(rosRoute('POST',path),false);
  for(const origin of ['https://127.0.0.1:57020','http://example.com:57020','http://a:b@127.0.0.1:57020','http://127.0.0.1:57020/x'])assert.throws(()=>rosGatewayOrigin(origin));
});
