import assert from "node:assert/strict";
import { existsSync, readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { spawnSync } from "node:child_process";

process.chdir(fileURLToPath(new URL("../", import.meta.url)));
const modes = new Set(process.argv.slice(2));
assert([...modes].every((mode) => ["--driver", "--build", "--bind-state", "--import-check"].includes(mode)),
  "Usage: node deploy/validate-docker.mjs [--driver] [--build] [--bind-state] [--import-check]");

function run(command, args, capture = false) {
  const result = spawnSync(command, args, {
    encoding: "utf8",
    stdio: capture ? ["ignore", "pipe", "inherit"] : "inherit",
  });
  if (result.error) throw result.error;
  assert.equal(result.status, 0, `${command} failed (exit ${result.status})`);
  return result.stdout?.trim();
}

const config = JSON.parse(run("docker", ["compose", "-f", "deploy/compose.yaml", "config", "--format", "json"], true));
assert.deepEqual(Object.keys(config.services).sort(), ["app", "go2rtc"]);
const { app, go2rtc } = config.services;
assert(/^[1-9]\d*:[1-9]\d*$/.test(app.user), "App UID/GID must be explicit nonroot numeric IDs");
assert.equal(go2rtc.user, "1000:1000");
for (const service of [app, go2rtc]) {
  assert.equal(service.read_only, true);
  assert.equal(service.restart, "unless-stopped");
  assert(service.cap_drop.includes("ALL"));
  assert(service.security_opt.includes("no-new-privileges:true"));
  assert(service.tmpfs.some((mount) => mount.startsWith("/tmp:")));
  assert(service.healthcheck.test.length > 1);
  for (const port of service.ports) {
    assert([57010, 57012].includes(Number(port.published)));
    assert.equal(Number(port.published), port.target === 57000 ? 57010 : 57012);
    assert(port.host_ip, "Publishing must use an explicit host address");
  }
}
assert.deepEqual(app.ports.map((p) => [p.target, p.protocol]), [[57000, "tcp"]]);
assert.deepEqual(go2rtc.ports.map((p) => [p.target, p.protocol]), [[57002, "tcp"], [57002, "udp"]]);
assert.equal(app.environment.ROBOCLAW_HOST, "0.0.0.0");
assert.equal(app.environment.ROBOCLAW_STATE_DIR, "/data");
assert.equal(app.environment.GO2RTC_ORIGIN, "http://go2rtc:57001");
const ollamaPort = Number(new URL(app.environment.OLLAMA_ORIGIN).port);
assert(ollamaPort >= 57000 && ollamaPort <= 57100, "Ollama must use the deployment port range");
assert(app.volumes.some((mount) => mount.target === "/data"));
const stateMount = app.volumes.find((mount) => mount.target === "/data");
if (stateMount.type === "volume") {
  assert.equal(app.user, "1000:1000", "Default named-volume ownership requires UID/GID 1000:1000; unset host UID/GID overrides");
}
assert(go2rtc.volumes.every((mount) => mount.read_only));
assert.equal(go2rtc.image, "roboclaw-go2rtc:local");
assert.equal(go2rtc.logging.driver, "none");
assert.equal(app.depends_on.go2rtc.condition, "service_healthy");
const dockerfile = readFileSync("deploy/Dockerfile", "utf8");
assert(dockerfile.includes("npm ci --omit=dev"));
assert(dockerfile.includes("npm run build"));
assert(dockerfile.includes('CMD ["node", "adapters/vendor/src/service-main.ts"]'));
assert.equal(readFileSync("deploy/Dockerfile.dockerignore", "utf8").split("\n")
  .find((line) => line && !line.startsWith("#")), "**");
console.log("Compose contract OK: 57010 HTTP and 57012 TCP/UDP published; native loopback ROS gateway via Docker host alias. No live services started.");

if (modes.has("--import-check")) {
  assert.equal(stateMount.type, "volume", "Import checks require the default named-volume configuration");
  const source = fileURLToPath(new URL("../.local", import.meta.url));
  const volume = `roboclaw-import-validation-${process.pid}-${Date.now()}`;
  run("docker", ["volume", "create", volume], true);
  try {
    run("docker", [
      "run", "--rm", "--network", "none", "--read-only", "--user", "0:0",
      "--cap-drop", "ALL", "--cap-add", "CHOWN", "--cap-add", "DAC_OVERRIDE",
      "--cap-add", "FOWNER", "--security-opt", "no-new-privileges",
      "--mount", `type=volume,source=${volume},target=/data`,
      "--mount", `type=bind,source=${source},target=/migration,readonly`,
      "--mount", `type=bind,source=${process.cwd()}/deploy,target=/checks,readonly`,
      "--entrypoint", "node", app.image, "/checks/import-state.mjs",
    ]);
    run("docker", [
      "run", "--rm", "--network", "none", "--read-only", "--user", "1000:1000",
      "--cap-drop", "ALL", "--security-opt", "no-new-privileges",
      "--tmpfs", "/data:rw,noexec,nosuid,size=1m,uid=1000,gid=1000,mode=0700",
      "--mount", `type=volume,source=${volume},target=/migration,readonly`,
      "--mount", `type=bind,source=${process.cwd()}/deploy,target=/checks,readonly`,
      "--entrypoint", "node", app.image, "/checks/check-bind-state.mjs",
    ]);
  } finally {
    run("docker", ["volume", "rm", volume], true);
  }
  console.log("Existing native state imported and verified in a temporary named volume; source unchanged, test volume removed.");
}

if (modes.has("--bind-state")) {
  assert.equal(stateMount.type, "bind", "Set ROBOCLAW_DATA_PATH to the existing absolute state directory");
  const [uid, gid] = app.user.split(":");
  run("docker", [
    "run", "--rm", "--network", "none", "--read-only", "--user", app.user,
    "--cap-drop", "ALL", "--security-opt", "no-new-privileges",
    "--tmpfs", "/tmp:rw,noexec,nosuid,size=64m",
    "--tmpfs", `/data:rw,noexec,nosuid,size=1m,uid=${uid},gid=${gid},mode=0700`,
    "--mount", `type=bind,source=${stateMount.source},target=/migration,readonly`,
    "--mount", `type=bind,source=${process.cwd()}/deploy,target=/checks,readonly`,
    "--entrypoint", "node", app.image, "/checks/check-bind-state.mjs",
  ]);
}

if (modes.has("--driver") || modes.has("--build")) {
  run("docker", ["version", "--format", "{{.Server.Version}}"]);
  run("docker", ["compose", "-f", "deploy/compose.yaml", "build", "go2rtc"]);
  run("docker", [
    "run", "--rm", "--network", "none", "--read-only", "--user", "1000:1000",
    "--cap-drop", "ALL", "--security-opt", "no-new-privileges",
    "--tmpfs", "/tmp:rw,noexec,nosuid,size=64m",
    "--tmpfs", "/config:rw,noexec,nosuid,size=1m,uid=1000,gid=1000",
    "--mount", `type=bind,source=${process.cwd()}/deploy,target=/checks,readonly`,
    "--mount", `type=bind,source=${process.cwd()}/deploy/go2rtc.yaml,target=/etc/go2rtc.yaml,readonly`,
    "-e", "ROBOCLAW_WEBRTC_HOST=127.0.0.1",
    "--entrypoint", "python3", go2rtc.image, "/checks/check-go2rtc.py",
  ]);
}

if (modes.has("--build")) {
  const manifest = JSON.parse(readFileSync("package.json", "utf8"));
  assert(manifest.scripts?.build, "Waiting for the parent's npm build script");
  assert(existsSync("adapters/vendor/src/service-main.ts"));
  assert(existsSync("apps/web/src"));
  app.image = "rrsz70-phase5:qualification";
  run("docker", ["build", "-f", "deploy/Dockerfile", "-t", app.image, "."]);
  const volume = `roboclaw-validation-${process.pid}-${Date.now()}`;
  run("docker", ["volume", "create", volume], true);
  try {
    for (const extra of [[], ["--existing-state"]]) {
      run("docker", [
        "run", "--rm", "--network", "none", "--read-only", "--user", "1000:1000",
        "--cap-drop", "ALL", "--security-opt", "no-new-privileges",
        "--tmpfs", "/tmp:rw,noexec,nosuid,size=64m",
        "--mount", `type=volume,source=${volume},target=/data`,
        "--mount", `type=bind,source=${process.cwd()}/deploy,target=/checks,readonly`,
        "-e", "GO2RTC_ORIGIN=http://127.0.0.1:57001",
        "-e", "OLLAMA_ORIGIN=http://127.0.0.1:57003",
        "--entrypoint", "node", app.image, "/checks/check-app.mjs", ...extra,
      ]);
    }
  } finally {
    run("docker", ["volume", "rm", volume], true);
  }
  console.log("Production image built and isolated startup verified; live camera/Ollama/host ports not exercised.");
}
