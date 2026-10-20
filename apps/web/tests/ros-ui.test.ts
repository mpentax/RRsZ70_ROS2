import test from 'node:test';
import assert from 'node:assert/strict';
import { createElement, act } from 'react';
import { dom, flush } from './setup-tsx.ts';
const {App,TaskList}=await import('../src/App.tsx');
const {SetupForm}=await import('../src/SetupForm.tsx');
const {SESSION_EXPIRED}=await import('../src/api.ts');

test('actual task rows expose shared ID, numeric phase/result and cancel same ID',async()=>{
  const {element,root,window}=dom();const id='b9e0e188-122b-449d-966b-f97b2bb63981';const ids:string[]=[];
  const task={id,kind:'pick',status:'executing',phase:3,feedback:{reason:'OBSERVING'},result:null};
  try {
    await act(async()=>root.render(createElement(TaskList,{tasks:[task],cancel:(id:string)=>ids.push(id)})));
    assert.match(element.textContent!,new RegExp(id));assert.match(element.textContent!,/phase 3/);
    await act(async()=>element.querySelector('button')!.click());assert.deepEqual(ids,[id]);
    await act(async()=>root.render(createElement(TaskList,{tasks:[{...task,status:'cancelled',phase:5,result:{outcome:{physical_completion:0}}}],cancel:(id:string)=>ids.push(id)})));
    assert.equal(element.querySelector('button')!.disabled,true);assert.match(element.textContent!,/physical_completion/);
  } finally {await act(async()=>root.unmount());await window.happyDOM.close();}
});

test('stop remains actionable while task start response stalls; expiry clears setup secrets',async t=>{
  const {element,root,window}=dom();const paths:string[]=[];let release:((r:Response)=>void)|undefined;
  t.mock.method(globalThis,'fetch',async(path:string,init:RequestInit)=>{
    paths.push(path);
    if(path==='/api/ros/tasks' && init?.method==='POST') return new Promise<Response>(resolve=>{release=resolve;});
    if(path==='/api/ros/tasks') return Response.json({tasks:[]});
    if(path==='/api/setup') return Response.json({device:null,accountConfigured:false,pinConfigured:false});
    if(path==='/api/ros/stop') return Response.json({cessation_observed:false,outcomes:[]});
    return Response.json({available:false,state:null,camera:null,arm:null,calibration:null});
  });
  try {
    await act(async()=>root.render(createElement(App,{session:Promise.resolve({authenticated:true})})));await flush();
    const start=Array.from(element.querySelectorAll('button')).find(b=>b.textContent==='Start Pick')!;
    await act(async()=>start.form!.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true})));await flush();
    assert.ok(release);const stop=Array.from(element.querySelectorAll('button')).find(b=>b.textContent==='STOP ALL')!;
    assert.equal(stop.disabled,false);await act(async()=>stop.click());await flush();assert.ok(paths.includes('/api/ros/stop'));
    await act(async()=>window.dispatchEvent(new window.Event(SESSION_EXPIRED)));await flush();
    assert.equal(element.querySelector('h1')?.textContent,'RoboClaw');assert.equal(element.querySelectorAll('input').length,1);
    assert.match(element.textContent!,/Camera PIN/);
    assert.equal(element.querySelector('input')!.value,'');
    release(Response.json({id:'b9e0e188-122b-449d-966b-f97b2bb63981',status:'accepted'}));await flush();
  } finally {await act(async()=>root.unmount());await window.happyDOM.close();}
});

test('navigation and garment patrol buttons dispatch pin_and_go, deliver_garment and patrol_collect tasks',async t=>{
  const {element,root,window}=dom();const started:unknown[]=[];
  t.mock.method(globalThis,'fetch',async(path:string,init:RequestInit)=>{
    if(path==='/api/ros/tasks' && init?.method==='POST'){
      const body=JSON.parse(String(init.body));started.push(body);
      return Response.json({id:'b9e0e188-122b-449d-966b-f97b2bb63981',kind:body.kind,status:'accepted',phase:1,feedback:null,result:null});
    }
    if(path==='/api/ros/tasks') return Response.json({tasks:[]});
    if(path==='/api/setup') return Response.json({device:null,accountConfigured:false,pinConfigured:false});
    return Response.json({available:true,state:{connected:true,battery_known:true,battery_fraction:0.9,evidence:{valid:true,stale:false}},camera:null,arm:null,calibration:null});
  });
  try {
    await act(async()=>root.render(createElement(App,{session:Promise.resolve({authenticated:true})})));await flush();
    const button=(text:string)=>Array.from(element.querySelectorAll('button')).find(b=>b.textContent===text)!;
    await act(async()=>button('Pin & Go').click());await flush();
    await act(async()=>button('Deliver Garment').click());await flush();
    await act(async()=>button('Patrol & Collect').click());await flush();
    assert.deepEqual(started,[
      {kind:'pin_and_go',goal:{target_x_m:0,target_y_m:0,max_duration_s:120,dry_run:true}},
      {kind:'deliver_garment',goal:{category:'any',drop_x_m:0,drop_y_m:0,max_duration_s:300,allow_towing:false,dry_run:true}},
      {kind:'patrol_collect',goal:{drop_x_m:0,drop_y_m:0,max_duration_s:600,max_items:5,allow_towing:false,dry_run:true}},
    ]);
  } finally {await act(async()=>root.unmount());await window.happyDOM.close();}
});

test('setup renders required web inputs and bounded expired-login recovery',async t=>{
  const {element,root,window}=dom();const submitted:unknown[]=[];let succeedLogin=false;
  t.mock.method(globalThis,'fetch',async(path:string,init:RequestInit)=>{
    submitted.push({path,body:JSON.parse(String(init.body))});
    if(path==='/api/account/code') return Response.json({sent:true});
    if(path==='/api/account/login' && succeedLogin) return Response.json({devices:[{id:'dev1',name:'Saros Z70',model:'roborock.vacuum.a179'}]});
    return Response.json({error:'Account login expired. Request a new code in web setup.'},{status:410});
  });
  try {
    await act(async()=>root.render(createElement(SetupForm,{setup:null,onUpdated:async()=>{}})));
    assert.ok(element.querySelector('input[type=email]'));assert.ok(element.querySelector('input[type=password]'));
    const form=element.querySelector('form')!;
    await act(async()=>form.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true})));await flush();
    const code=element.querySelector('input[pattern="[0-9]{6}"]')!;assert.ok(code);
    await act(async()=>code.closest('form')!.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true})));await flush();
    assert.match(element.textContent!,/Account login expired/);assert.equal(element.querySelector('input[pattern="[0-9]{6}"]'),null);
    assert.equal((submitted[1] as {body:{code:string}}).body.code,'');
    succeedLogin=true;
    await act(async()=>form.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true})));await flush();
    const code2=element.querySelector('input[pattern="[0-9]{6}"]')!;
    await act(async()=>code2.closest('form')!.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true})));await flush();
    assert.match(element.textContent!,/Settings > Product Info > Wi-Fi Name/i);
  } finally {await act(async()=>root.unmount());await window.happyDOM.close();}
});
