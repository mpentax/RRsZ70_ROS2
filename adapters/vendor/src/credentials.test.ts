import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtemp, rm, stat, chmod, symlink, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { saveDevice, loadDevice, summarizeStatus } from "./credentials.ts";

const device = {
  duid: "test-device", name: "Test Z70", model: "roborock.vacuum.a179",
  host: "192.0.2.10", localKey: "0123456789abcdef", protocolVersion: "1.0",
};

test("credentials have private permissions, survive reload and can be replaced atomically", async () => {
  const root = await mkdtemp(join(tmpdir(), "roboclaw-test-"));
  const directory = join(root, "private");
  try {
    await saveDevice(directory, device);
    assert.equal((await stat(directory)).mode & 0o777, 0o700);
    assert.equal((await stat(join(directory, "device.json"))).mode & 0o777, 0o600);
    assert.deepEqual(await loadDevice(directory), device);
    const updated = { ...device, name: "Updated Z70" };
    await saveDevice(directory, updated);
    assert.deepEqual(await loadDevice(directory), updated);
    await chmod(join(directory, "device.json"), 0o644);
    await assert.rejects(loadDevice(directory), /private regular file/);
  } finally {
    await rm(root, { recursive: true, force: true });
  }
});

test("credential persistence rejects a symlink directory", async () => {
  const root = await mkdtemp(join(tmpdir(), "roboclaw-test-"));
  try {
    await symlink(root, join(root, "link"));
    await assert.rejects(saveDevice(join(root, "link"), device), /real directory/);
  } finally {
    await rm(root, { recursive: true, force: true });
  }
});

test("status summary permits only selected scalar fields", () => {
  assert.deepEqual(summarizeStatus([{ battery: 90, state: 8, localKey: "secret", fw_ver: {} }]), {
    battery: 90, state: 8,
  });
  assert.deepEqual(summarizeStatus(null), {});
  assert.deepEqual(summarizeStatus([]), {});
});

test("malformed credential JSON never echoes its contents", async () => {
  const root = await mkdtemp(join(tmpdir(), "roboclaw-test-"));
  try {
    await writeFile(join(root, "device.json"), '{"localKey":"sensitive-value", broken}', { mode: 0o600 });
    await assert.rejects(loadDevice(root), (error: unknown) =>
      error instanceof Error && !error.message.includes("sensitive-value") && error.message.includes("invalid JSON"));
  } finally {
    await rm(root, { recursive: true, force: true });
  }
});
