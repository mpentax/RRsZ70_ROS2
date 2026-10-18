/** Operator-confirmed migration after a successful explicit local handshake.
 * Cloud pv is retained verbatim. No network request or command replay occurs.
 */
import { SecretStore } from "./secret-store.ts";
import { localProtocol } from "./credentials.ts";
import { isIP } from "node:net";
const [host, model, protocol] = process.argv.slice(2);
if (!host || !isIP(host) || !model) throw new Error("Expected verified host, model and local protocol");
const selected = localProtocol(protocol);
const store = new SecretStore();
await store.initialize();
const pairing = await store.loadPairing();
const device = pairing?.device ?? await store.loadDevice();
if (!device || device.host !== host || device.model !== model) throw new Error("Selected device does not match verified target");
const updated = { ...device, localProtocol: selected };
// Preserve the atomic pairing (including private account credentials) as one record.
if (pairing) await store.savePairing({ ...pairing, device: updated });
else await store.saveDevice(updated);
console.log(JSON.stringify({ migrated: true, localProtocol: selected, cloudMetadataPreserved: true, atomicPairing: !!pairing }));
