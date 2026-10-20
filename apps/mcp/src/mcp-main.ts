import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { fileURLToPath } from "node:url";
import { createRoboClawMcp, mcpServiceOrigin, readMcpToken, RoboClawMcpClient } from "./mcp-server.ts";

try {
  const origin = mcpServiceOrigin(process.env.ROBOCLAW_MCP_ORIGIN ?? "http://127.0.0.1:57010");
  const token = await readMcpToken(process.env.ROBOCLAW_MCP_TOKEN_FILE ?? fileURLToPath(new URL("../../../.local/service-token", import.meta.url)));
  const server = createRoboClawMcp(new RoboClawMcpClient(origin, token));
  await server.connect(new StdioServerTransport());
} catch {
  // stdout is exclusively MCP protocol; never log private paths, tokens, or upstream responses.
  console.error("RoboClaw MCP could not start; check local origin and token file.");
  process.exitCode = 1;
}
