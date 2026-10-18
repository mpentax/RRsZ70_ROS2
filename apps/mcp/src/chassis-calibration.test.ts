import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RoboClawMcpClient, mcpServiceOrigin } from './mcp-server.ts';

test('MCP calibration comes from shared typed ROS state with unchanged evidence',async()=> {
  const calibration={evidence:{sequence:14,stale:true},camera_x_known:true,camera_x_m:.28};
  const client=new RoboClawMcpClient(mcpServiceOrigin('http://127.0.0.1:57010'),'x'.repeat(43),async url=>{
    assert.equal(new URL(String(url)).pathname,'/api/ros/state');return Response.json({calibration});
  });
  assert.deepEqual(await client.request('/api/ros/state'),{calibration});
});
