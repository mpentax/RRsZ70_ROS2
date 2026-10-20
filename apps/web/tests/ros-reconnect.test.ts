import test from 'node:test';
import assert from 'node:assert/strict';
import { createElement, act } from 'react';
import { dom, flush } from './setup-tsx.ts';
const {App}=await import('../src/App.tsx');

test('rendered web reconnects read-only and clears only connection errors',async t=>{
  const {window,element,root}=dom();
  let available=false;const writes:string[]=[];
  t.mock.method(globalThis,'fetch',async(path:string,init:RequestInit)=>{
    if(init.method==='POST')writes.push(path);
    if(path==='/api/ros/tasks')return Response.json({tasks:[]});
    if(path==='/api/setup')return Response.json({device:null,accountConfigured:false,pinConfigured:false});
    if(!available)return Response.json({error:'ROS_UNAVAILABLE',dispatch:'not_sent'},{status:503});
    return Response.json({available:true,state:{connected:true,battery_known:true,battery_fraction:.75,evidence:{stale:false}}});
  });
  try {
    await act(async()=>{root.render(createElement(App,{session:Promise.resolve({authenticated:true})}));});
    await flush(30);
    assert.match(element.textContent!,/ROS_UNAVAILABLE/);
    available=true;
    await flush(1600);
    assert.match(element.textContent!,/ROS driver available/);assert.match(element.textContent!,/75%/);
    assert.doesNotMatch(element.textContent!,/ROS_UNAVAILABLE/);assert.deepEqual(writes,[]);
  } finally {await act(async()=>root.unmount());await window.happyDOM.close();}
});
