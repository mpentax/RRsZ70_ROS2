import { createHash, randomUUID } from "node:crypto";
import { constants } from "node:fs";
import { link, open, unlink } from "node:fs/promises";
import { join, parse, resolve } from "node:path";
import { MAX_MAP_BYTES } from "./photo-protocol.ts";
import { ensurePrivateDir, hasCode, syncPrivateDir } from "./secret-store.ts";
import { summarizeV1Map } from "./v1-map-summary.ts";

/** Save a verified V1 map binary into the private state directory. */
export async function savePrivateV1Map(raw: Buffer, stateDirectory: string): Promise<{ sha256: string; bytes: number; saved: boolean }> {
  // Validate before touching disk; never print raw bytes, geometry, filesystem errors, or private paths.
  if (!Buffer.isBuffer(raw) || raw.length > MAX_MAP_BYTES) throw new Error("Invalid private V1 map.");
  try { summarizeV1Map(raw); } catch { throw new Error("Invalid private V1 map."); }
  const sha256 = createHash("sha256").update(raw).digest("hex");
  if (typeof stateDirectory !== "string" || !stateDirectory.trim() || stateDirectory.includes("\0")) {
    throw new Error("Cannot save private V1 map.");
  }
  const root = resolve(stateDirectory);
  if (root === parse(root).root) throw new Error("Cannot save private V1 map.");
  const directory = join(root, "maps");
  const destination = join(directory, `v1-${sha256}.bin`);
  const temporary = join(directory, `.v1-${randomUUID()}.tmp`);
  try {
    await ensurePrivateDir(root, "Unsafe private map directory.");
    await ensurePrivateDir(directory, "Unsafe private map directory.");
    let saved = false;
    const handle = await open(temporary,
      constants.O_WRONLY | constants.O_CREAT | constants.O_EXCL | constants.O_NOFOLLOW, 0o600);
    try {
      await handle.chmod(0o600);
      await handle.writeFile(raw);
      await handle.sync();
    } finally { await handle.close(); }
    try { await link(temporary, destination); saved = true; }
    catch (error) {
      if (!hasCode(error, "EEXIST")) throw error;
      // Existing name may be tampered with: refuse anything except a private,
      // regular, byte-identical file. O_NOFOLLOW protects against symlink swaps.
      const existing = await open(destination, constants.O_RDONLY | constants.O_NOFOLLOW | constants.O_NONBLOCK);
      try {
        const stat = await existing.stat();
        if (!stat.isFile() || stat.nlink !== 1 || (stat.mode & 0o7777) !== 0o600 || stat.size !== raw.length) throw new Error("Unsafe private map file.");
        const bytes = await existing.readFile();
        if (!bytes.equals(raw)) throw new Error("Private map hash collision or changed file.");
      } finally { await existing.close(); }
    }
    // Remove the temporary hardlink before fsync and reporting success.
    await unlink(temporary);
    await syncPrivateDir(directory);
    return { sha256, bytes: raw.length, saved };
  } catch { throw new Error("Cannot save private V1 map."); }
  finally { try { await unlink(temporary); } catch { /* absent or cleanup failure cannot expose map bytes */ } }
}
