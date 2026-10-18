import assert from "node:assert/strict";
import { existsSync, readdirSync, readFileSync, statSync, writeFileSync } from "node:fs";
import { spawn } from "node:child_process";
import { setTimeout } from "node:timers/promises";

assert.equal(process.getuid(), 1000);
assert.equal(process.versions.node.split(".")[0], "24");
assert(existsSync("/app/dist/web/index.html"));
assert(existsSync("/app/node_modules/mqtt"));
for (const path of [".local", ".env", "web", "deploy", "node_modules/vite", "node_modules/typescript"]) {
  assert(!existsSync(`/app/${path}`), `Unexpected runtime input: ${path}`);
}
function checkSource(path) {
  for (const entry of readdirSync(path, { withFileTypes: true })) {
    assert(!entry.isSymbolicLink(), "Unexpected source symlink");
    assert(!/(^\.|\.test\.tsx?$|\.(?:pem|key|jpe?g|png|db|log)$)/i.test(entry.name),
      `Unexpected private/test input: ${entry.name}`);
    assert(!["device.json", "camera.json", "service-token", "motion-hold.json", "keys", "data", "logs", "images"]
      .includes(entry.name), `Unexpected persisted state: ${entry.name}`);
    if (entry.isDirectory()) checkSource(`${path}/${entry.name}`);
  }
}
checkSource("/app/adapters/vendor/src");
checkSource("/app/apps/mcp/src");
checkSource("/app/packages");
const holdPath = "/data/motion-hold.json";
const tokenPath = "/data/service-token";
const hold = '{"reason":"deployment-validation-only"}';
const previousToken = existsSync(tokenPath) ? readFileSync(tokenPath, "utf8") : null;
if (process.argv.includes("--existing-state")) {
  assert(previousToken, "Named volume did not preserve the service token across containers");
  assert.equal(readFileSync(holdPath, "utf8"), hold);
} else {
  assert.equal(previousToken, null, "Validation requires a new, isolated state volume");
  writeFileSync(holdPath, hold, { mode: 0o600, flag: "wx" });
}

const child = spawn(process.execPath, ["adapters/vendor/src/service-main.ts"], {
  cwd: "/app",
  // Do not print generated bootstrap tokens or upstream error messages.
  stdio: "ignore",
});
try {
  let healthy = false;
  for (let attempt = 0; attempt < 100; attempt++) {
    assert.equal(child.exitCode, null, "Backend exited before becoming healthy");
    try {
      const response = await fetch("http://127.0.0.1:57000/healthz", {
        signal: AbortSignal.timeout(1000),
      });
      await response.body?.cancel();
      healthy = response.ok;
    } catch (error) {
      if (!(error instanceof TypeError) && error.name !== "TimeoutError") throw error;
    }
    if (healthy) break;
    await setTimeout(100);
  }
  assert(healthy, "Backend did not become healthy within the startup deadline");
  const page = await fetch("http://127.0.0.1:57000/");
  assert.equal(page.status, 200);
  assert((await page.text()).includes('<div id="root">'));
  assert(readdirSync("/data").length > 0, "Backend did not persist bootstrap state");
  assert.equal(readFileSync(holdPath, "utf8"), hold, "Startup changed the motion hold");
  if (previousToken) assert.equal(readFileSync(tokenPath, "utf8"), previousToken);
  const token = readFileSync(tokenPath, "utf8");
  const unauthenticated = await fetch("http://127.0.0.1:57000/api/state");
  assert.equal(unauthenticated.status, 401);
  await unauthenticated.body?.cancel();
  const stateResponse = await fetch("http://127.0.0.1:57000/api/state", {
    headers: { authorization: `Bearer ${token}` },
  });
  assert.equal(stateResponse.status, 200);
  const state = await stateResponse.json();
  assert.equal(state.device, null);
  assert.equal(state.camera.pinCached, false);
  assert.equal(state.connection.state, "disconnected");
  assert.equal(state.api.commandOwner, "ROS");

  for (const name of readdirSync("/data")) {
    const path = `/data/${name}`;
    if (statSync(path).isFile()) {
      assert.equal(statSync(path).mode & 0o077, 0, "State files must be owner-only");
      assert(readFileSync(path).length > 0, "Empty bootstrap state file");
    }
  }
  console.log("Node 24 production app: nonroot/read-only, frontend, health, private state and motion hold OK");
} finally {
  const exited = new Promise((resolve) => child.once("exit", resolve));
  child.kill("SIGTERM");
  if (child.exitCode === null) {
    await Promise.race([exited, setTimeout(5000)]);
    if (child.exitCode === null && child.signalCode === null) child.kill("SIGKILL");
  }
}
