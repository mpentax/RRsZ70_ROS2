'use strict';
const {test}=require('node:test');const assert=require('node:assert/strict');const {replay}=require('./replay_editor.cjs');
const path=process.env.EDITOR_JS;
test('pinned vendor source generates exact commands, bounds and bridge; network/native denied',{skip:!path},()=>{
 const r=replay(path);const c=r.cases;
 assert.deepEqual(c.m4_up_1.start_bridge,{cmd_ids:[{id:1,cmd_id:'arm_joints',value:[0,1,0,0],step:1}],program_id:null});
 assert.deepEqual(c.m4_up_2.commands,[{id:1,cmd_id:'arm_joints',value:[0,1,0,0],step:2}]);
 assert.deepEqual(c.m4_down_2.commands,[{id:1,cmd_id:'arm_joints',value:[0,2,0,0],step:2}]);
 assert.deepEqual(c.deploy_m4_1.commands,[{id:1,cmd_id:'arm_out'},{id:2,cmd_id:'arm_joints',value:[0,1,0,0],step:1}]);
 assert.deepEqual(c.m6_release.commands[0].value,[0,0,0,1]);assert.deepEqual(c.m6_grip.commands[0].value,[0,0,0,2]);
 assert.equal(c.m6_grip.commands[0].step,1);
 assert.deepEqual(r.bounds.arm_m4.step.options.map(x=>x[1]),Array.from({length:10},(_,i)=>String(i+1)));
 assert.equal(r.bounds.arm_m4.dir.value,'1');assert.equal(r.bounds.arm_m4.step.value,'1');
 assert.deepEqual(r.bounds.delay.Time,{value:1,min:1,max:100,step:1,defaultValue:1,unit:''});
 assert.deepEqual(r.delay_modal.map(x=>[x.value,x.open]),[[1,'block'],[1,'none'],[100,'none'],[100,'block'],[2,'none'],[1,'none']]);
 assert.deepEqual(r.normalAttempts,['missing original Blockly serializer']);
 assert.deepEqual(r.trapAttempts.slice(1),['fetch','XHR','WS','native','SDK']);
 assert.match(r.saveMissing.error,/missing original Blockly serializer/);
});
