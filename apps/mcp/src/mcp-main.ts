import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import { dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { createRoboClawMcp, mcpServiceOrigin, readMcpToken, RoboClawMcpClient } from "./mcp-server.ts";
import { exitOnSourceChange, trackInflight } from "./mcp-reload.ts";

try {
  const origin = mcpServiceOrigin(process.env.ROBOCLAW_MCP_ORIGIN ?? "http://127.0.0.1:57010");
  const token = await readMcpToken(process.env.ROBOCLAW_MCP_TOKEN_FILE ?? fileURLToPath(new URL("../../../.local/service-token", import.meta.url)));
  const server = createRoboClawMcp(new RoboClawMcpClient(origin, token));
  const transport = new StdioServerTransport();
  await server.connect(transport);
  // Long-lived hosts (Hermes desktop) never respawn this child on their own; restart when the code changes.
  if (process.env.ROBOCLAW_MCP_RELOAD !== "off")
    exitOnSourceChange({ dir: process.env.ROBOCLAW_MCP_WATCH_DIR ?? dirname(fileURLToPath(import.meta.url)), inflight: trackInflight(transport) });
} catch {
  // stdout is exclusively MCP protocol; never log private paths, tokens, or upstream responses.
  console.error("RoboClaw MCP could not start; check local origin and token file.");
  process.exitCode = 1;
}
