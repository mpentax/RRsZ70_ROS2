// Turn right in ~30 deg steps until the colour detector finds the target (default yellow). RoboClaw MCP only.
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { StdioClientTransport } from "@modelcontextprotocol/sdk/client/stdio.js";
import { writeFileSync, mkdirSync, appendFileSync } from "node:fs";
import { execFileSync } from "node:child_process";
import { fileURLToPath } from "node:url";
const repo = fileURLToPath(new URL('../../', import.meta.url)), color = process.argv[2] ?? "yellow";
const out = `${repo}/calibration/pickup/${new Date().toISOString().replace(/[-:]/g, "").slice(0, 15)}Z-scan`; mkdirSync(out, { recursive: true });
const c = new Client({ name: "scan", version: "0" });
await c.connect(new StdioClientTransport({ command: process.execPath, args: [`${repo}/apps/mcp/src/mcp-main.ts`], cwd: repo,
  env: { ...process.env, ROBOCLAW_MCP_ORIGIN: "http://127.0.0.1:57010", ROBOCLAW_MCP_RELOAD: "off" } }));
const t0 = Date.now(), say = (...a) => console.log(((Date.now() - t0) / 1000).toFixed(1) + "s", ...a);
const call = async (name, args = {}) => { const r = await c.callTool({ name, arguments: args }); const text = r.content?.filter(x => x.type === "text").map(x => x.text).join("") ?? ""; let j; try { j = JSON.parse(text); } catch { j = { text }; } const img = r.content?.find(x => x.type === "image"); return { isError: !!r.isError, ...j, ...(img ? { imageData: img.data } : {}) }; };
const rpc = (method, params, timeoutMs) => call("robot_rpc", { method, params, ...(timeoutMs ? { timeoutMs } : {}) });
const sleep = ms => new Promise(r => setTimeout(r, ms));
async function look(i) {
  const f = await call("get_camera_frame", {}); const file = `${out}/scan-${String(i).padStart(2, "0")}.jpg`;
  writeFileSync(file, Buffer.from(f.imageData, "base64"));
  const res = JSON.parse(execFileSync(`${process.env.HOME}/.pixi/bin/pixi`, ["run", "-q", "python", `${repo}/calibration/pickup/find.py`, file], { cwd: repo, encoding: "utf8" }).trim().split("\n").pop());
  const objs = Array.isArray(res.objects) ? res.objects : (res.objects?.objects ?? []);
  appendFileSync(`${out}/log.jsonl`, JSON.stringify({ t: Date.now(), i, file, objects: objs, raw: res.raw.filter(b => b.color === color) }) + "\n");
  return { objs, raw: res.raw.filter(b => b.color === color) };
}
let seq = 1, found = null;
try {
  const s = (await rpc("get_status", [])).result?.[0]; say("status", s?.state, "battery", s?.battery, "err", s?.error_code);
  if (s?.error_code) throw new Error("robot error");
  await call("select_camera", { camera: "front" });
  let r = await look(0); say(`look 0: ${color} objects=${JSON.stringify(r.objs.filter(o => o.color === color)).slice(0, 200)} raw=${r.raw.length}`);
  if (r.objs.some(o => o.color === color)) found = 0;
  if (found === null) { await rpc("app_rc_start", []); await sleep(5500); }
  for (let i = 1; found === null && i <= 12; i++) {
    const mv = await rpc("app_rc_move", [{ velocity: 0, omega: -0.5, duration: 1600, seqnum: seq++ }]);
    if (mv.dispatch !== "sent") say("move outcome", mv.dispatch, "- checking by looking");
    await sleep(3600); await rpc("app_rc_stop", []);
    r = await look(i);
    say(`look ${i} (~${i * 30} deg right): ${color} objects=${JSON.stringify(r.objs.filter(o => o.color === color)).slice(0, 240)} raw blobs=${r.raw.map(b => `${Math.round(b.pixel[0])},${Math.round(b.pixel[1])}:${Math.round(b.area_px)}`).join(" ")}`);
    if (r.objs.some(o => o.color === color)) found = i;
  }
} finally {
  await rpc("app_rc_stop", []); await rpc("app_rc_end", []);
  say(found === null ? "not found" : `FOUND at look ${found}`, "->", out);
  await c.close();
}
