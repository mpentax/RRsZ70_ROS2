import {test} from 'node:test';
import assert from 'node:assert/strict';
import type {IncomingMessage, ServerResponse} from 'node:http';
import {ServiceAuth} from './service-auth.ts';
import {loadServiceConfig} from './service-config.ts';

test('localhost browser setup works through a loopback-published container network peer',()=>{
  const config=loadServiceConfig({ROBOCLAW_HOST:'0.0.0.0',ROBOCLAW_PUBLIC_ORIGIN:'http://127.0.0.1:57010'});
  const auth=new ServiceAuth('test-service-token-'.repeat(3),config);
  const req={headers:{host:'127.0.0.1:57010',origin:config.publicOrigin},socket:{remoteAddress:'172.20.0.1'}} as IncomingMessage;
  let cookie='';const res={setHeader(name:string,value:string){assert.equal(name,'Set-Cookie');cookie=value;}} as ServerResponse;
  auth.setup(req,res);
  assert.match(cookie,/HttpOnly; SameSite=Strict; Max-Age=1200/);
});
