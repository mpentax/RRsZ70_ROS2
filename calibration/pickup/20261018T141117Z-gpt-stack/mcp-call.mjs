// One deliberate RoboClaw MCP call; never talks directly to vendor/HTTP.
// Server owns credential loading. Every command and reply is retained here.
import {Client} from '@modelcontextprotocol/sdk/client/index.js';
import {StdioClientTransport} from '@modelcontextprotocol/sdk/client/stdio.js';
import {appendFile,copyFile,writeFile} from 'node:fs/promises';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
const directory=dirname(fileURLToPath(import.meta.url));
const [name,raw='{}',captureName]=process.argv.slice(2);
const allowed=new Set(['get_state','get_camera_frame','get_arm_estimate','list_tasks','get_task','start_task','robot_rpc','stop','set_arm_anchor','select_camera']);
if(!allowed.has(name)) throw new Error('Unknown tool');
const args=JSON.parse(raw);
const client=new Client({name:'supervised-gpt-stack',version:'1.0'});
const transport=new StdioClientTransport({command:process.execPath,args:[resolve(directory,'../../../apps/mcp/src/mcp-main.ts')],env:{...process.env,ROBOCLAW_MCP_RELOAD:'off'},stderr:'pipe'});
const record=async(value)=>appendFile(resolve(directory,'mcp-calls.jsonl'),JSON.stringify({at:new Date().toISOString(),...value})+'\n');
try{
 await client.connect(transport);
 const tools=await client.listTools();
 if(!tools.tools.some(t=>t.name===name)) throw new Error('Tool not present');
 await record({event:'request',name,args});
 const result=await client.callTool({name,arguments:args},undefined,{timeout:45000});
 await record({event:'reply',name,result});
 let value=result;
 if(result.content?.[0]?.text){try{value=JSON.parse(result.content[0].text);}catch{}}
 const image=result.content?.find(part=>part.type==='image' && part.mimeType==='image/jpeg');
 if(name==='get_camera_frame' && captureName && (value.imagePath || image)){
  if(!/^[A-Za-z0-9_-]+\.jpg$/.test(captureName)) throw new Error('Invalid capture filename');
  if(value.imagePath) await copyFile(value.imagePath,resolve(directory,captureName));
  else await writeFile(resolve(directory,captureName),Buffer.from(image.data,'base64'));
  await writeFile(resolve(directory,captureName+'.json'),JSON.stringify(value,null,2));
  value={...value,retainedImagePath:resolve(directory,captureName)};
 }
 console.log(JSON.stringify(value,null,2));
 if(result.isError) process.exitCode=1;
}finally{await client.close();}
