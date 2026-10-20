import test from 'node:test';
import assert from 'node:assert/strict';
import {createService, type ServiceStore} from './service.ts';
import {loadServiceConfig} from './service-config.ts';
import {request as httpRequest} from 'node:http';

const jpeg = Buffer.from([255,216,255,192,0,11,8,4,176,6,64,1,1,17,0,255,218,0,2,255,217]);
test('session stop releases preview and fences pending opens and queued reads even across resume', async t => {
  const calls:string[]=[]; let finish!:()=>void; let configuring!:()=>void;
  const entered = new Promise<void>(r=>{configuring=r;});
  const pending = new Promise<void>(r=>{finish=r;});
  const token='f'.repeat(43);
  const store:ServiceStore={async initialize(){},async serviceToken(){return token;},
    async loadPairing(){return {device:{duid:'fixture',host:'127.0.0.1',localKey:'0123456789abcdef',name:'fake',model:'roborock.vacuum.a179',protocolVersion:'L01'},camera:{account:{user:'fixture',secret:'fake-secret',key:'fake-key',mqttUrl:'ssl://mqtt-eu.roborock.com:8883'},pin:'1234'},home:{deviceId:'fixture',homeId:1,realApi:'https://api-eu.roborock.com',user:'fixture',secret:'s',hawkKey:'k',rooms:[]}};},
    async savePairing(){}};
  const config={...loadServiceConfig(),port:0};
  const service=await createService({store,config,robotFactory:()=>({connection:{state:'connected',connectedAt:1},close(){},async request(method){calls.push(method);return method==='get_camera_status'?[0]:['ok'];}}),
    driver:{async configure(){calls.push('configure');configuring();await pending;},async frame(){calls.push('frame');return jpeg;},async offer(){return {type:'answer',sdp:'v=0'};},close(){calls.push('close');},async preload(){calls.push('preload');},async unload(){calls.push('unload');}}});
  await service.start();t.after(async()=>{finish();await service.stop();});
  const origin=service.origin;
  const post=(path:string,body:unknown)=>fetch(origin+path,{method:'POST',headers:{Authorization:['Bearer',token].join(' '),'Content-Type':'application/json'},body:JSON.stringify(body)});
  const opening=post('/api/camera/frame',{});await entered;
  let slowHold!:ReturnType<typeof httpRequest>;
  const holdResponse=new Promise<number>((resolve,reject)=>{
    slowHold=httpRequest(origin+'/api/agent/camera/hold',{method:'POST',headers:{Authorization:`Bearer ${token}`,'Content-Type':'application/json','Content-Length':Buffer.byteLength('{"enabled":true}')}},response=>{response.resume();resolve(response.statusCode!);});
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
