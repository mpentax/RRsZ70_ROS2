import test from 'node:test';
import assert from 'node:assert/strict';
import type { HTMLInputElement } from 'happy-dom';
import { createElement, act } from 'react';
import { dom, flush } from './setup-tsx.ts';
const {App}=await import('../src/App.tsx');
const {SESSION_EXPIRED}=await import('../src/api.ts');

test('first-run browser offers setup without ROS and returns to PIN sign-in on completion',async t=>{
  const {window,element,root,button}=dom(t);const paths:string[]=[];let completed=false;
  t.mock.method(globalThis,'fetch',async(path:string,init:RequestInit)=>{
    paths.push(path);
    if(path==='/api/session' && init.method==='POST')return Response.json({authenticated:false,scope:'setup',setupAvailable:true});
    if(path==='/api/session')return Response.json({authenticated:false,scope:completed?null:'setup',setupAvailable:!completed});
    if(path==='/api/setup')return Response.json({device:{id:'robot1',name:'Test Z70',model:'a179',host:'127.0.0.1',localProtocol:'L01'},accountConfigured:true,pinConfigured:false});
    if(path==='/api/camera/pin'){completed=true;return Response.json({pinCached:true});}
    return Response.json({error:'ROS_UNAVAILABLE'},{status:503});
  });
  await act(async()=>root.render(createElement(App,{session:Promise.resolve({authenticated:false,scope:null,setupAvailable:true})})));await flush();
  const start=button('Start local setup');assert.ok(start);
  await act(async()=>start.click());await flush();
  assert.match(element.textContent!,/Initial local setup/);assert.match(element.textContent!,/Test Z70/);
  assert.ok(element.querySelector('input[type=email]'));assert.doesNotMatch(element.textContent!,/Start Pick|STOP ALL|ROS_UNAVAILABLE/);
  assert.equal(paths.some(p=>p.startsWith('/api/ros/')),false);
  const pin=element.querySelector('input[placeholder="4–12 digits (1–9)"]')!;
  await act(async()=>pin.closest('form')!.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true})));await flush();
  assert.match(element.textContent!,/Setup complete. Sign in with your camera PIN/);
  assert.equal(element.querySelector('input[type=email]'),null);assert.equal(element.querySelectorAll('input').length,1);
  assert.equal(element.querySelector('input')!.value,'');
  assert.equal(paths.some(p=>p.startsWith('/api/ros/')),false);
});

test('stale first-run setup conflict refreshes session and offers usable PIN sign-in',async t=>{
  const {window,element,root,button}=dom(t);const requests:string[]=[];
  t.mock.method(globalThis,'fetch',async(path:string,init:RequestInit)=>{
    requests.push(`${init.method} ${path}`);
    if(path==='/api/session' && init.method==='POST'){
      const body=JSON.parse(init.body as string);
      if(body.setup){assert.deepEqual(body,{setup:true});return Response.json({}, {status:409});}
      assert.deepEqual(body,{pin:'123456'});
      return Response.json({authenticated:true,scope:'control',setupAvailable:false});
    }
    if(path==='/api/session')return Response.json({authenticated:false,scope:null,setupAvailable:false});
    if(path==='/api/setup')return Response.json({device:null,accountConfigured:true,pinConfigured:true});
    return Response.json({error:'ROS_UNAVAILABLE'},{status:503});
  });
  await act(async()=>root.render(createElement(App,{session:Promise.resolve({authenticated:false,scope:null,setupAvailable:true})})));
  await act(async()=>button('Start local setup')!.click());await flush();
  assert.deepEqual(requests,['POST /api/session','GET /api/session']);
  assert.equal(Boolean(button('Start local setup')),false);
  const pin=element.querySelector<HTMLInputElement>('input[type=password]');assert.ok(pin);
  // Invoke React's registered change handler so controlled state changes too.
  const props=Object.entries(pin).find(([key])=>key.startsWith('__reactProps$'))![1];
  await act(async()=>{pin.value='123456';props.onChange({target:pin});});
  const signIn=button('Sign in')!;
  assert.equal(signIn.disabled,false);
  await act(async()=>pin.closest('form')!.dispatchEvent(new window.Event('submit',{bubbles:true,cancelable:true})));await flush();
  assert.equal(requests.filter(r=>r==='POST /api/session').length,2);
  assert.match(element.textContent!,/ROS unavailable/);
  assert.ok(button('Sign out'));
});

test('expired first-run setup clears secret inputs and offers setup again without a reload',async t=>{
  const {window,element,root,button}=dom(t);
  t.mock.method(globalThis,'fetch',async(path:string)=>Response.json(path==='/api/session'
    ? {authenticated:false,scope:null,setupAvailable:true}
    : {device:null,accountConfigured:false,pinConfigured:false}));
  await act(async()=>root.render(createElement(App,{session:Promise.resolve({authenticated:false,scope:'setup' as const,setupAvailable:true})})));
  assert.ok(element.querySelector('input[type=email]'));
  await act(async()=>window.dispatchEvent(new window.Event(SESSION_EXPIRED)));
  await flush();
  assert.equal(element.querySelector('input[type=email]'),null);
  assert.ok(button('Start local setup'));
});
