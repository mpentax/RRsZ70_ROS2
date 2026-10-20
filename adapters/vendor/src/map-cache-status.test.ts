import test from "node:test";
import assert from "node:assert/strict";
import { mkdtemp, lstat, readFile, realpath, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { createHash } from "node:crypto";
import { SecretStore } from "./secret-store.ts";
import { mapMetadataFromRead, validatedMapMetadata } from "./map-cache-status.ts";

function synthetic(): Buffer {
  const header = Buffer.alloc(20); header.write("rr"); header.writeUInt16LE(20, 2);
  const image = Buffer.alloc(25); image.writeUInt16LE(2); image.writeUInt16LE(24, 2);
  image.writeUInt32LE(1, 4); image.writeUInt32LE(1, 16); image.writeUInt32LE(1, 20); image[24] = 7;
  const digest = Buffer.alloc(28); digest.writeUInt16LE(1024); digest.writeUInt16LE(8, 2); digest.writeUInt32LE(20, 4);
  const raw = Buffer.concat([header, image, digest]); raw.writeUInt32LE(raw.length - 20, 4);
  createHash("sha1").update(raw.subarray(0, -20)).digest().copy(raw, raw.length - 20);
  return raw;
}

test("private map metadata persists bound to selected device, checks bytes and private modes", async t => {
  const root = await realpath(await mkdtemp(join(tmpdir(), "roboclaw-map-metadata-")));
  t.after(() => rm(root, { recursive: true, force: true }));
  const store = new SecretStore(join(root, "state"));
  await store.initialize();
  const raw = synthetic(), sha256 = createHash("sha256").update(raw).digest("hex");
  const result = { raw, sha256, bytes: raw.length, mapIndex: 0, mapSequence: 0 };
  await store.saveMapBytes(raw);
  await store.saveMapCache(mapMetadataFromRead("private-device", result));
  const loaded = await store.loadMapRasterSource();
  assert.ok(loaded);
  const metadata = validatedMapMetadata(loaded.metadata);
  assert.equal(metadata.deviceId, "private-device");
  assert.equal(metadata.sha256, sha256);
  assert.equal((await lstat(join(root, "state", "map-cache.json"))).mode & 0o777, 0o600);
  assert.equal((await lstat(join(root, "state", "maps"))).mode & 0o777, 0o700);
  assert.equal((await lstat(join(root, "state"))).mode & 0o777, 0o700);
  const file = join(root, "state", "maps", `v1-${sha256}.bin`);
  const corrupted = Buffer.from(await readFile(file)); corrupted[25] ^= 1;
  await writeFile(file, corrupted);
  await assert.rejects(store.loadMapRasterSource(), /Cannot load private map raster source/);
});
