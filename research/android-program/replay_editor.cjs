'use strict';
// Offline vendor-inline replay. No dependencies, CDN loading, native dispatch or asset redistribution.
// VM is defense-in-depth, not a general hostile-code security boundary. Only digest-pinned source is run.
const fs = require('node:fs');
const vm = require('node:vm');
const crypto = require('node:crypto');
const assert = require('node:assert/strict');
const SOURCE_SHA = 'e60813d955a7e78c66493cc605a13ea870f8d8baa12f308147216d39d97dc048';
function replay(path) {
 const source = fs.readFileSync(path, 'utf8');
 assert.equal(crypto.createHash('sha256').update(source).digest('hex'), SOURCE_SHA);
 // All objects/functions exposed to vendor code are created inside the VM; no host callbacks/require/process.
 const ctx = vm.createContext(Object.create(null), {codeGeneration:{strings:false, wasm:false}});
 vm.runInContext(`
 const attempts = [], messages = [], nodes = new Map();
 const deny = name => function(){ attempts.push(name); throw new Error('DENIED '+name); };
 globalThis.fetch=deny('fetch'); globalThis.XMLHttpRequest=deny('XHR'); globalThis.WebSocket=deny('WS');
 globalThis.Worker=deny('Worker'); globalThis.importScripts=deny('importScripts');
 globalThis.callMethod=deny('native'); globalThis.SDK={callMethod:deny('SDK')};
 globalThis.setTimeout=()=>0; globalThis.clearTimeout=()=>{};
 globalThis.console={log(){},warn(){}};
 function element(id){ if(!nodes.has(id)) nodes.set(id,{style:{},value:'',listeners:{},addEventListener(k,f){this.listeners[k]=f;},focus(){},querySelector(){return null;},cloneNode(){return element(id+'clone');},replaceWith(e){nodes.set(id,e);}}); return nodes.get(id); }
 globalThis.document={getElementById:element,createElement:deny('DOM creation')};
 globalThis.localStorage={setItem(){},length:0};
 globalThis.window=globalThis; window.visualViewport={addEventListener(){}}; window.addEventListener=()=>{};
 window.ReactNativeWebView={postMessage(x){messages.push(x);}};
 class Field { constructor(v){this.value=v;} getValue(){return this.value;} setValue(v){this.value=v;} }
 class FieldDropdown extends Field {constructor(options){super(options[0][1]);this.options=options;}}
 const ws={roots:[],getBlocksByType(t){return this.roots.filter(b=>b.type===t);},addChangeListener(){}};
 globalThis.Blockly={Field,FieldDropdown,Blocks:{},Msg:{},BlockSvg:class{},WorkspaceSvg:class{},Flyout:class{},
 JavaScript:{forBlock:{}},inputs:{Align:{CENTRE:0}},fieldRegistry:{register(){}},inject(){return ws;},
 serialization:{workspaces:{save:deny('missing original Blockly serializer')}}};
 function block(type,fields={}){return {type,fields,next:null,getFieldValue(k){return this.fields[k];},getNextBlock(){return this.next;}};}
 function sequence(spec){const bs=spec.map(s=>block(s[0],s[1]));bs.forEach((b,i)=>b.next=bs[i+1]||null);ws.roots=[{type:'custom_procedure',getInputTargetBlock(){return bs[0]||null;}}];}
 function defaults(type){const fields={}; const input={setAlign(){return this;},appendField(f,n){if(n)fields[n]=f;return this;}};
 const b={appendDummyInput(){return input;},setPreviousStatement(){},setNextStatement(){},setTooltip(){},setHelpUrl(){},setColour(){}};
 Blockly.Blocks[type].init.call(b);return fields;}
 `,ctx,{timeout:1000});
 vm.runInContext(source,ctx,{timeout:1000});
 const result = vm.runInContext(`
 const cases={};
 for(const [name,spec] of Object.entries({m4_up_1:[['arm_m4',{dir:'1',step:'1'}]],m4_up_2:[['arm_m4',{dir:'1',step:'2'}]],m4_down_2:[['arm_m4',{dir:'2',step:'2'}]],deploy_m4_1:[['arm_out',{}],['arm_m4',{dir:'1',step:'1'}]],m6_release:[['arm_m6',{dir:'1'}]],m6_grip:[['arm_m6',{dir:'2'}]],delay_default:[['delay',{Time:1}]]})){
 sequence(spec);const commands=getCommands();const before=messages.length;element('btnStart').listeners.click();cases[name]={commands,start_bridge:JSON.parse(messages[before])}; }
 const bounds={};for(const type of ['arm_m4','arm_m6','arm_move','delay']){bounds[type]=defaults(type);}
 // Exercise vendor modal validation, not a reconstructed clamp.
 localizationStrings={};const field=defaults('delay').Time;const modal=[];
 for(const raw of ['0','1','100','101','2.9','bad']){field.showEditor_();element('angleInput').value=raw;element('confirmBtn').listeners.click();modal.push({raw,value:field.getValue(),open:element('angleModal').style.display});}
 const saveMissing={};try{saveTemplate('offline',cases.m4_up_1.commands);}catch(e){saveMissing.error=e.message;}
 const normalAttempts=attempts.slice();
 for(const expr of [()=>fetch('x'),()=>new XMLHttpRequest(),()=>new WebSocket('x'),()=>callMethod('x'),()=>SDK.callMethod('x')]){try{expr();}catch(e){}}
 JSON.stringify({provenance:{source_sha256:'${SOURCE_SHA}',execution:'original complete inline source, synthetic Blockly block/DOM adapters',blocks_serialization:'UNAVAILABLE: original external Blockly missing; never fabricated'},cases,bounds,delay_modal:modal,saveMissing,normalAttempts,trapAttempts:attempts});
 `,ctx,{timeout:1000});
 return JSON.parse(result);
}
if(require.main===module){const r=replay(process.argv[2]); console.log(JSON.stringify(r,null,2));}
module.exports={replay};
