import { test, type TestContext } from 'node:test';
import assert from 'node:assert/strict';
import { createServer, request as httpRequest } from 'node:http';
import { createService, type ServiceOptions, type ServiceStore } from './service.ts';
import { loadServiceConfig } from './service-config.ts';
import type { CameraLogin } from './cloud.ts';

const token = 'test-service-token-'.repeat(3);
const device = {duid:'robot1',host:'127.0.0.1',localKey:'fake-local-key-abcdef',name:'Test Z70',model:'a179',protocolVersion:'L01'};
const camera = {account:{user:'test-user',secret:'fake-account-secret',key:'fake-account-key',mqttUrl:'mqtts://example.invalid'},pin:''};
const home = {deviceId:'robot1',homeId:1,realApi:'https://example.invalid',user:'test-user',secret:'fake-home-secret',hawkKey:'fake-hawk-key',rooms:[]};
const login: CameraLogin = {devices:[device],account:camera.account,home};
type Pairing = NonNullable<Awaited<ReturnType<ServiceStore['loadPairing']>>>;
async function fixture(t:TestContext, initial: Pairing|null = null, overrides:Partial<ServiceOptions> = {}) {
  let saved:Pairing|null = initial; let writes = 0, privateCalls = 0;
  const backend = createServer((req,res)=>{privateCalls++; assert.equal(req.headers.authorization,`Bearer ${token}`);res.setHeader('Content-Type','application/json');res.end(JSON.stringify({available:true}));});
  await new Promise<void>(resolve=>backend.listen(0,'127.0.0.1',resolve));
  t.after(()=>new Promise<void>(resolve=>backend.close(()=>resolve())));
  const address = backend.address();assert.ok(address && typeof address==='object');
  const store:ServiceStore = {async initialize(){},async serviceToken(){return token;},async loadPairing(){return saved;},async savePairing(value){saved=value;writes++;}};
  const config={...loadServiceConfig({}),port:0};
  const service=await createService({store,config,rosOrigin:`http://127.0.0.1:${address.port}`,robotFactory:()=>({connection:{state:'disconnected',connectedAt:null},close(){},async request(){privateCalls++;return ['ok'];}}),cloudFactory:()=>({async requestCode(){},async loginCameraWithCode(){return login;}}),...overrides});
  await service.start();t.after(()=>service.stop());
  const bind=service.server.address();assert.ok(bind && typeof bind==='object');
  const origin=`http://127.0.0.1:${bind.port}`;config.allowedHosts.push(`127.0.0.1:${bind.port}`);config.publicOrigin=origin;
  async function request(path:string, body?:unknown, cookie?:string, headers:Record<string,string|undefined>={}) {
    // node:http preserves custom Host headers; fetch intentionally rewrites Host.
    const result = await new Promise<{status:number;body:any;cookie:string|undefined}>((resolve,reject)=>{
      const req=httpRequest(origin+path,{method:body===undefined?'GET':'POST',headers:{Origin:origin,...(body===undefined?{}:{'Content-Type':'application/json'}),...(cookie?{Cookie:cookie}:{}),...headers}},res=>{
        const chunks:Buffer[]=[];res.on('data',chunk=>chunks.push(chunk));res.on('error',reject);
        res.on('end',()=>{
          try {
            const text=Buffer.concat(chunks).toString();
            for(const secret of [token,device.localKey,camera.account.secret,camera.account.key,home.hawkKey,home.secret]) assert.equal(text.includes(secret),false,`response leaked ${secret}`);
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
  const f=await fixture(t);
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
  const f=await fixture(t,{device,camera,home},{cloudFactory:()=>({async requestCode(){},async loginCameraWithCode(){entered.resolve();return delayed.promise;}})});
  const first=await f.request('/api/session',{setup:true}), second=await f.request('/api/session',{setup:true});
  await f.request('/api/account/code',{email:'user@example.com',region:'eu'},first.cookie);
  const pending=f.request('/api/account/login',{code:'123456'},first.cookie);await entered.promise;
  assert.equal((await f.request('/api/camera/pin',{pin:'3456'},second.cookie)).status,200);
  delayed.resolve(login);
  assert.equal((await pending).status,401);
  for(const cookie of [first.cookie,second.cookie]) {
    assert.equal((await f.request('/api/session',undefined,cookie)).body.scope,null);
    assert.equal((await f.request('/api/camera/pin',{pin:'4567'},cookie)).status,401);
  }
  assert.equal(f.saved?.camera.pin,'3456');assert.equal(f.writes,1);assert.equal(f.privateCalls,0);
});

test('setup retains Host and Origin protection and cannot be issued for a remote public origin',async t=>{
  const f=await fixture(t);
  for(const headers of [{Origin:'https://hostile.invalid'},{Origin:''},{'Sec-Fetch-Site':'cross-site'},{Host:'hostile.invalid'}])
    assert.equal((await f.request('/api/session',{setup:true},undefined,headers)).status,403,JSON.stringify(headers));
  const session=await f.request('/api/session',{setup:true});
  for(const headers of [{Origin:'https://hostile.invalid'},{Origin:''},{'Sec-Fetch-Site':'cross-site'}]) {
    assert.equal((await f.request('/api/account/code',{email:'user@example.com',region:'eu'},session.cookie,headers)).status,403);
    assert.equal((await f.request('/api/camera/pin',{pin:'2345'},session.cookie,headers)).status,403);
  }
  assert.equal((await f.request('/api/ros/state',undefined,session.cookie,{Authorization:'Bearer wrong'})).status,401);
  f.config.allowedHosts.push('remote.example');f.config.publicOrigin='http://remote.example';
  assert.equal((await f.request('/api/session',{setup:true},undefined,{Host:'remote.example',Origin:'http://remote.example'})).status,403);
  assert.equal(f.writes,0);assert.equal(f.privateCalls,0);
});

test('saving camera PIN with device selection closes bootstrap immediately and preserves configured login',async t=>{
  const f=await fixture(t);const first=await f.request('/api/session',{setup:true});const other=await f.request('/api/session',{setup:true});
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
  const f=await fixture(t,null,{cloudFactory:()=>({async requestCode(){throw new Error(`private user@example.com 123456 ${token} ${camera.account.secret}`);},async loginCameraWithCode(){return login;}})});
  const session=await f.request('/api/session',{setup:true});
  const result=await f.request('/api/account/code',{email:'user@example.com',region:'eu'},session.cookie);
  assert.equal(result.status,502);assert.doesNotMatch(JSON.stringify(result.body),/user@example.com|123456|private/);
});
