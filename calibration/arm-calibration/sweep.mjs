// Arm calibration sweep (Claude). RoboClaw MCP only; no VLM in the loop.
// node sweep.mjs <outdir>   (reads calibration/arm-calibration/sweep.json)
import { Client } from "@modelcontextprotocol/sdk/client/index.js";
import { StdioClientTransport } from "@modelcontextprotocol/sdk/client/stdio.js";
import { readFileSync, writeFileSync, appendFileSync, copyFileSync, mkdirSync } from "node:fs";
import { fileURLToPath } from "node:url";

const repo = fileURLToPath(new URL('../../', import.meta.url));
const out = process.argv[2]; mkdirSync(out, { recursive: true });
const plan = JSON.parse(readFileSync(`${repo}/calibration/arm-calibration/sweep.json`, "utf8"));
const c = new Client({ name: "arm-sweep", version: "0" });
await c.connect(new StdioClientTransport({ command: process.execPath, args: [`${repo}/apps/mcp/src/mcp-main.ts`], cwd: repo,
  env: { ...process.env, ROBOCLAW_MCP_ORIGIN: "http://127.0.0.1:57010", ROBOCLAW_MCP_RELOAD: "off" } }));
const t0 = Date.now(), ts = () => ((Date.now() - t0) / 1000).toFixed(1);
const log = (o) => { appendFileSync(`${out}/log.jsonl`, JSON.stringify({ t: Date.now(), ...o }) + "\n"); };
const say = (...a) => console.log(ts() + "s", ...a);
const call = async (name, args = {}) => { const r = await c.callTool({ name, arguments: args }); const text = r.content?.filter(x => x.type === "text").map(x => x.text ?? "").join("") ?? ""; let j; try { j = JSON.parse(text); } catch { j = { text: text.slice(0, 300) }; } const img = r.content?.find(x => x.type === "image"); return { isError: !!r.isError, ...j, ...(img ? { imageData: img.data } : {}) }; };
const rpc = (method, params, timeoutMs) => call("robot_rpc", { method, params, ...(timeoutMs ? { timeoutMs } : {}) });
const sleep = ms => new Promise(r => setTimeout(r, ms));
const status = async () => (await rpc("get_status", [])).result?.[0] ?? {};
let stowed = false;
async function stow(reason) {
  if (stowed) return; stowed = true;
  say("stowing:", reason); log({ event: "stow", reason });
  const r = await rpc("app_arm_in_compartment", {}, 30000); log({ event: "stow_reply", r });
  await sleep(12000);
  await call("select_camera", { camera: "front" }).catch(() => {});
}
let interrupted = false;
process.on("SIGINT", () => { interrupted = true; });

async function check(where) {
  const s = await status();
  if (s.error_code) throw new Error(`robot error ${s.error_code} at ${where}`);
  if (s.state !== 39) throw new Error(`left pickup mode (state ${s.state}) at ${where}`);
  return s;
}

try {
  let s = await status(); say("start state", s.state, "battery", s.battery, "err", s.error_code); log({ event: "start", status: s });
  if (s.error_code) throw new Error("robot error at start");
  if (s.state !== 39) { log({ event: "pickup", r: await rpc("app_pick_up_enter_exit", { flag: 2 }) }); await sleep(2000); }
  log({ event: "arm_out", r: await rpc("app_arm_out_compartment", {}, 30000) });
  for (let i = 0; i < 20; i++) { await sleep(3000); s = await status(); if (s.common_status & 0x100) break; }
  if (!(s.common_status & 0x100)) throw new Error("arm never reported ready");
  say("arm ready", s.common_status); log({ event: "ready", status: s, estimate: await call("get_arm_estimate") });

  for (const [pass, camera] of [["A", "gripper"], ["B", "front"]]) {
    const sel = await call("select_camera", { camera }); say(`pass ${pass}: select ${camera} ->`, JSON.stringify(sel).slice(0, 80));
    if (sel.isError) throw new Error(`select ${camera} failed`);
    await call("get_camera_frame", {});            // absorb the slow first frame after a switch
    let joints = [150, 45, 0], n = 0;
    const seq = pass === "A" ? plan : plan;          // same path both passes; each ends back at the deployed pose
    for (const step of seq) {
      if (interrupted) throw new Error("ctrl-c");
      if (step.move) {
        const r = await rpc("app_arm_move", { m: step.move }, 15000);
        log({ event: "move", pass, step: step.move, dispatch: r.dispatch, response: r.response, result: r.result });
        if (r.dispatch !== "sent" || r.response !== "received") throw new Error(`move ${JSON.stringify(step.move)} outcome ${r.dispatch}/${r.response}`);
        joints = joints.map((v, k) => v + 10 * step.move[k]);
        await sleep(2500);
      } else {
        await sleep(1500);
        const st = await check(`capture ${n}`);
        const f = await call("get_camera_frame", {});
        const est = await call("get_arm_estimate");
        const name = `${pass}-${String(n).padStart(2, "0")}-${camera}.jpg`;
        if (f.imageData) writeFileSync(`${out}/${name}`, Buffer.from(f.imageData, "base64"));
        else if (f.imagePath) copyFileSync(f.imagePath, `${out}/${name}`);
        else throw new Error("frame has no image data");
        log({ event: "capture", pass, n, planned: step.capture, tracked: joints, wristModel: step.wrist, file: name, camera: f.camera,
              size: f.dimensions, estimate: { state: est.state, joints: est.joints, wrist: est.wrist }, common: st.common_status });
        say(`${pass}${n} joints=${joints} est=${est.state}/${JSON.stringify(est.joints)} cam=${f.camera} ${f.dimensions?.width}x${f.dimensions?.height}`);
        n++;
      }
    }
  }
  await stow("sweep complete");
} catch (e) {
  say("ABORT:", e.message); log({ event: "abort", error: e.message });
  await stow("abort: " + e.message);
} finally {
  await c.close();
}
