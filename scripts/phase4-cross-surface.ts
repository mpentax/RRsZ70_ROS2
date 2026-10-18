// Offline integration harness, launched only by isolated fake-DDS pytest.
import assert from 'node:assert/strict';
import { Client } from '@modelcontextprotocol/sdk/client/index.js';
import { InMemoryTransport } from '@modelcontextprotocol/sdk/inMemory.js';
import { createService, type ServiceStore } from '../adapters/vendor/src/service.ts';
import { loadServiceConfig } from '../adapters/vendor/src/service-config.ts';
import { createRoboClawMcp, RoboClawMcpClient } from '../apps/mcp/src/mcp-server.ts';
import { startTask, listTasks, getTask, cancelTask, stopRobot } from '../apps/web/src/ros.ts';
import { Window } from 'happy-dom';
import { createElement, act } from 'react';
import { createRoot } from 'react-dom/client';
import './phase4-test-tsx.ts';
const {App}=await import('../apps/web/src/App.tsx');

const gateway = process.argv[2];
assert.ok(gateway && /^http:\/\/127\.0\.0\.1:\d+$/.test(gateway));
const token='f'.repeat(43), pin='1234';
const store:ServiceStore={async initialize(){},async serviceToken(){return token;},async loadDevice(){return null;},async saveDevice(){},async loadCamera(){return {account:{user:'u',secret:'s',key:'k',mqttUrl:'ssl://mqtt.roborock.com:8883'},pin};},async saveCamera(){}};
const config={...loadServiceConfig(),port:0};
const service=await createService({store,config,rosOrigin:gateway});
await service.start();
const address=service.server.address();assert.ok(address && typeof address==='object');
const origin=`http://127.0.0.1:${address.port}`;config.allowedHosts.push(`127.0.0.1:${address.port}`);config.publicOrigin=origin;
const nativeFetch=globalThis.fetch;
const login=await nativeFetch(origin+'/api/session',{method:'POST',headers:{Origin:origin,'Content-Type':'application/json'},body:JSON.stringify({pin})});
assert.equal(login.status,200);const cookie=login.headers.get('set-cookie')!.split(';')[0]!;
// Browser api.ts uses relative URLs and same-origin cookies; the MCP client uses bearer.
const window=new Window({url:origin});
Object.assign(globalThis,{window,document:window.document,HTMLElement:window.HTMLElement,IS_REACT_ACT_ENVIRONMENT:true});
const container=window.document.createElement('div');window.document.body.append(container);
const root=createRoot(container as unknown as Element);
globalThis.fetch=async (input,init)=>{
  if(typeof input==='string' && input.startsWith('/')) return nativeFetch(origin+input,{...init,headers:{...init?.headers,Origin:origin,Cookie:cookie}});
  return nativeFetch(input,init);
};
const server=createRoboClawMcp(new RoboClawMcpClient(new URL(origin),token));
const mcp=new Client({name:'phase4-cross-surface',version:'1'});
const [a,b]=InMemoryTransport.createLinkedPair();
async function tool(name:string,args:Record<string,unknown>={}) {
  const result=await mcp.callTool({name,arguments:args});assert.notEqual(result.isError,true,JSON.stringify(result));
  return JSON.parse((result.content as {text:string}[])[0]!.text);
}
async function until(check:()=>Promise<any>) {
  for(let n=0;n<150;n++){const value=await check();if(value) return value;await new Promise(resolve=>setTimeout(resolve,20));}
  throw new Error('cross-surface condition deadline');
}
try {
  await server.connect(b);await mcp.connect(a);
  await act(async()=>root.render(createElement(App,{session:Promise.resolve({authenticated:true})})));
  const first=await tool('start_task',{task:{kind:'pick',goal:{color:'yellow',dry_run:true}}});
  const fromWeb=await until(async()=>{const row=await getTask(first.id);return row.status==='succeeded' && row;});
  assert.equal(fromWeb.id,first.id);assert.equal(fromWeb.phase,5);
  assert.deepEqual(await tool('get_task',{id:first.id}),fromWeb);
  assert.ok((await listTasks()).tasks.some(row=>row.id===first.id));
  await until(async()=>{await act(async()=>{await new Promise(resolve=>setTimeout(resolve,30));});return container.querySelector(`[data-task-id="${first.id}"]`);});
  assert.match(container.querySelector(`[data-task-id="${first.id}"]`)!.textContent!,/succeeded, phase 5/);
  assert.match(container.querySelector(`[data-task-id="${first.id}"]`)!.textContent!,/dry_run_valid/);
  const webStarted=await startTask({kind:'pick',goal:{color:'wait',dry_run:true}});
  await until(async()=>{const row=await tool('get_task',{id:webStarted.id});return row.phase===3;});
  const before=Date.now();const stopped=await stopRobot() as {cessation_observed:boolean};
  assert.equal(stopped.cessation_observed,false);assert.ok(Date.now()-before<2000);
  assert.equal((await tool('cancel_task',{id:webStarted.id})).id,webStarted.id);
  await until(async()=>{const row=await getTask(webStarted.id);return row.status==='cancelled';});
  const mcpStarted=await tool('start_task',{task:{kind:'pick',goal:{color:'wait',dry_run:true}}});
  await until(async()=>{const row=await getTask(mcpStarted.id);return row.phase===3;});
  await until(async()=>{await act(async()=>{await new Promise(resolve=>setTimeout(resolve,30));});return container.querySelector(`[data-task-id="${mcpStarted.id}"] button`);});
  await act(async()=>{(container.querySelector(`[data-task-id="${mcpStarted.id}"] button`) as unknown as {click:()=>void}).click();});
  await until(async()=>{const row=await tool('get_task',{id:mcpStarted.id});return row.status==='cancelled';});
  const failed=await startTask({kind:'pick',goal:{color:'fail',dry_run:true}});
  await until(async()=>{const row=await tool('get_task',{id:failed.id});return row.status==='failed';});
  // A task submitted by the actual rendered web form is visible to MCP.
  const existing=new Set((await tool('list_tasks')).tasks.map((t:{id:string})=>t.id));
  const startButton=Array.from(container.querySelectorAll('button')).find(b=>b.textContent==='Start Pick')!;
  await act(async()=>{startButton.form!.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true}));});
  const uiTask=await until(async()=>{await act(async()=>{await new Promise(resolve=>setTimeout(resolve,30));});return (await tool('list_tasks')).tasks.find((t:{id:string;status:string})=>!existing.has(t.id)&&t.status==='succeeded');});
  assert.equal(uiTask.phase,5);
  console.log(JSON.stringify({cross_surface:true,rendered_ui:true,web_form_to_mcp:true,cancel_both_directions:true,stop_during_task:true,shared_ids:[first.id,webStarted.id,mcpStarted.id,failed.id,uiTask.id],failed_result:true}));
} finally {
  await act(async()=>root.unmount());await window.happyDOM.close();
  globalThis.fetch=nativeFetch;await mcp.close();await server.close();await service.stop();
}
