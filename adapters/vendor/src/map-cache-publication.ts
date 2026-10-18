import { savePrivateV1Map } from "./private-map-cache.ts";
import { mapMetadataFromRead, type PrivateMapMetadata } from "./map-cache-status.ts";

/** Publish CLI opt-in map bytes first, then selected-device metadata.
 * A failed/partial write can leave unreferenced private bytes, not a false cache claim.
 */
export async function publishCliMapCache(store: { saveMapCache(metadata: PrivateMapMetadata): Promise<void> },
  stateDirectory: string, deviceId: string, read: {
    raw: Buffer; bytes: number; sha256: string; mapIndex: number; mapSequence: number;
  }): Promise<{ sha256: string; bytes: number; saved: boolean }> {
  const metadata = mapMetadataFromRead(deviceId, read);
  const cache = await savePrivateV1Map(read.raw, stateDirectory);
  if (cache.sha256 !== metadata.sha256 || cache.bytes !== metadata.bytes) {
    throw new Error("Private map read differs from validated cache bytes.");
  }
  await store.saveMapCache(metadata);
  return cache;
}
