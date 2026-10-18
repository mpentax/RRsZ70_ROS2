import assert from "node:assert/strict";
import { chown, chmod, lstat, readFile, readdir, writeFile } from "node:fs/promises";

assert.equal(process.getuid(), 0, "This one-shot volume import requires the documented root capabilities");
const source = await lstat("/migration");
assert(source.isDirectory() && !source.isSymbolicLink(), "Migration source must be a directory");
assert.equal(source.mode & 0o7777, 0o700, "Migration source must have mode 0700");
assert.deepEqual(await readdir("/data"), [], "Destination must be an empty named volume; refusing overwrite");
const names = ["device.json", "camera.json", "service-token", "motion-hold.json"];
for (const name of names) {
  const info = await lstat(`/migration/${name}`);
  assert(info.isFile() && !info.isSymbolicLink(), "Migration source must contain regular private state files");
  assert.equal(info.mode & 0o7777, 0o600, "Migration source files must have mode 0600");
  assert.equal(info.uid, source.uid, "Migration source ownership is inconsistent");
  assert(info.size <= 16 * 1024, "Unexpectedly large migration file");
}
for (const name of names) {
  const contents = await readFile(`/migration/${name}`);
  await writeFile(`/data/${name}`, contents, { flag: "wx", mode: 0o600 });
  assert.deepEqual(await readFile(`/data/${name}`), contents, "State copy mismatch");
  await chown(`/data/${name}`, 1000, 1000);
}
await chmod("/data", 0o700);
await chown("/data", 1000, 1000);
console.log("Imported private state and any arm interlock verbatim into an empty UID-1000 volume; source unchanged.");
