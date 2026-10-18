import assert from "node:assert/strict";
import { lstat, readFile, writeFile } from "node:fs/promises";
import { SecretStore } from "/app/adapters/vendor/src/secret-store.ts";

const uid = process.getuid();
assert(uid > 0, "State must run as a nonroot user");
const directory = await lstat("/migration");
assert(directory.isDirectory() && !directory.isSymbolicLink(), "State must be a real directory");

assert.equal(directory.mode & 0o7777, 0o700, "State directory must have mode 0700");
const names = ["device.json", "camera.json", "service-token", "motion-hold.json"];
for (const name of names) {
  const info = await lstat(`/migration/${name}`);
  assert(info.isFile() && !info.isSymbolicLink(), "State files must be regular files");

  assert.equal(info.mode & 0o7777, 0o600, "State files must have mode 0600");
  assert(info.size <= 16 * 1024, "Unexpectedly large state file");
  // Validate the real store against a private tmpfs copy, never the source bind.
  await writeFile(`/data/${name}`, await readFile(`/migration/${name}`), {
    mode: 0o600, flag: "wx",
  });
}
const store = new SecretStore("/data");
await store.initialize();
assert(await store.loadDevice(), "Existing device state was not accepted");
assert(await store.loadCamera(), "Existing camera state was not accepted");
assert(await store.motionHeld(), "Existing motion hold was not preserved");
assert.equal(await store.serviceToken(), await readFile("/migration/service-token", "utf8"));
for (const name of names) {
  assert.deepEqual(await readFile(`/data/${name}`), await readFile(`/migration/${name}`),
    "Store initialization changed the persisted state");
}
console.log(`Existing bind state accepted as UID ${uid}: modes, device, camera, token and motion hold verified; source mounted read-only.`);
