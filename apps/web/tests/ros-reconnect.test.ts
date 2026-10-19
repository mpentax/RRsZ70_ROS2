import test from 'node:test';
import assert from 'node:assert/strict';
import {Window} from 'happy-dom';
import {createElement,act} from 'react';
import {createRoot} from 'react-dom/client';
import './setup-tsx.ts';
const {App}=await import('../src/App.tsx');

test('rendered web reconnects read-only and clears only connection errors',async t=>{
  const window=new Window();Object.assign(globalThis,{window,document:window.document,IS_REACT_ACT_ENVIRONMENT:true});
  const element=window.document.createElement('div');const root=createRoot(element as unknown as Element);
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
    await act(async()=>{await new Promise(resolve=>setTimeout(resolve,30));});
    assert.match(element.textContent!,/ROS_UNAVAILABLE/);
    available=true;
    await act(async()=>{await new Promise(resolve=>setTimeout(resolve,1600));});
    assert.match(element.textContent!,/ROS driver available/);assert.match(element.textContent!,/75%/);
    assert.doesNotMatch(element.textContent!,/ROS_UNAVAILABLE/);assert.deepEqual(writes,[]);
  } finally {await act(async()=>root.unmount());await window.happyDOM.close();}
});
