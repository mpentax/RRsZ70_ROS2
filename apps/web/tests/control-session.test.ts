import test from 'node:test';
import assert from 'node:assert/strict';
import { createElement, act } from 'react';
import { dom, flush } from './setup-tsx.ts';
const {App}=await import('../src/App.tsx');

test('rendered session stop remains usable during a stalled command and requires explicit resume',async t=>{
  const {window,element,root}=dom();
  const writes:{path:string;body:unknown}[]=[];let enabled=true;let finish!:()=>void;
  const pending=new Promise<void>(r=>{finish=r;});
  t.mock.method(globalThis,'fetch',async(path:string,init:RequestInit)=>{
    if(init.method==='POST')writes.push({path,body:JSON.parse(init.body as string)});
    if(path==='/api/ros/control/session'){
      enabled=JSON.parse(init.body as string).enabled;
      return Response.json({enabled,streams_released:!enabled,cessation_observed:false,outcomes:[]});
    }
    if(path==='/api/ros/tasks'&&init.method==='POST'){await pending;return Response.json({id:'fake'});}
    if(path==='/api/ros/tasks')return Response.json({tasks:[]});
    if(path==='/api/setup')return Response.json({device:null,accountConfigured:false,pinConfigured:false});
    return Response.json({available:true,session_enabled:enabled});
  });
  const button=(text:string)=>Array.from(element.querySelectorAll('button')).find(b=>b.textContent===text);
  try{
    await act(async()=>{root.render(createElement(App,{session:Promise.resolve({authenticated:true})}));});
    await act(async()=>{button('M3 + nominal step')!.click();});
    const stop=button('Stop control & streams');assert.ok(stop,'dedicated session stop must exist');
    assert.equal(stop.disabled,false);
    await act(async()=>{stop.click();});
    assert.deepEqual(writes.at(-1),{path:'/api/ros/control/session',body:{enabled:false}});
    assert.equal(button('M3 + nominal step')!.disabled,true);
    assert.ok(button('Resume control & streams'));
    await flush(1600);
    assert.equal(writes.filter(w=>w.path==='/api/ros/control/session').length,1,'polling must not resume');
    await act(async()=>{button('Resume control & streams')!.click();});
    assert.deepEqual(writes.at(-1),{path:'/api/ros/control/session',body:{enabled:true}});
  }finally{finish();await act(async()=>{await pending;root.unmount();});await window.happyDOM.close();}
});
