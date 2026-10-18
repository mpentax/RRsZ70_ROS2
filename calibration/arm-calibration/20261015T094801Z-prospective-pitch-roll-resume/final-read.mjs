import {Client} from '@modelcontextprotocol/sdk/client/index.js';
import {StdioClientTransport} from '@modelcontextprotocol/sdk/client/stdio.js';
import {writeFileSync} from 'node:fs';
const c=new Client({name:'prospective-final-read',version:'1'});
const repo='/Users/mpentax/Code/RoboRockZ70-Connector';
await c.connect(new StdioClientTransport({command:process.execPath,args:[`${repo}/src/mcp-main.ts`],cwd:repo,env:{...process.env,ROBOCLAW_MCP_RELOAD:'off'}}));
try {const r=await c.callTool({name:'get_state',arguments:{}});const data=JSON.parse(r.content.filter(x=>x.type==='text').map(x=>x.text).join(''));writeFileSync(new URL('final-state.json',import.meta.url),JSON.stringify(data,null,2));console.log(JSON.stringify(data));}finally{await c.close();}
