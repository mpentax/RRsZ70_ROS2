import { randomUUID } from "node:crypto";
import { connect } from "node:tls";
import type { CameraAccount, DeviceCredentials } from "./cloud.ts";
import { mqttOptions } from "./cloud-control.ts";

export function parseCameraConnack(bytes: Buffer) {
  if (bytes.length !== 4 || bytes[0] !== 0x20 || bytes[1] !== 2 || bytes[2] > 1 || bytes[3] > 5) return { valid: false };
  return { valid: true, accepted: bytes[3] === 0, sessionPresent: bytes[2] === 1, returnCode: bytes[3] };
}

export function cameraMqttConnect(clientId: string, username: string, password: string) {
  const field = (value: string) => { const data = Buffer.from(value); const size = Buffer.alloc(2); size.writeUInt16BE(data.length); return Buffer.concat([size, data]); };
  const body = Buffer.concat([field("MQTT"), Buffer.from([4, 0xc2, 0, 30]), field(clientId), field(username), field(password)]);
  let size = body.length; const length: number[] = [];
  do { let digit = size % 128; size = Math.floor(size / 128); if (size) digit |= 128; length.push(digit); } while (size);
  return Buffer.concat([Buffer.from([0x10, ...length]), body]);
}

/** MQTT CONNECT only: no subscriptions, camera preview, or robot RPCs. Outputs no credentials. */
export async function cameraBrokerDiagnostic(account: CameraAccount, device: DeviceCredentials, connector: typeof connect = connect) {
  const broker = new URL(account.mqttUrl);
  const credentials = { broker: account.mqttUrl.replace(/^ssl:/, "mqtts:"), user: account.user, secret: account.secret,
    accountKey: account.key, deviceId: device.duid, localKey: device.localKey };
  const auth = mqttOptions(credentials);
  const probe = (clientId: string) => new Promise<Record<string, unknown>>(resolve => {
    let stage = "tls_connect", done = false; const chunks: Buffer[] = []; let bytes = 0;
    const socket = connector({ host: broker.hostname, port: Number(broker.port), servername: broker.hostname, rejectUnauthorized: true });
    const finish = (result: Record<string, unknown>) => { if (done) return; done = true; clearTimeout(timer); socket.destroy(); resolve(result); };
    const timer = setTimeout(() => finish({ valid: false, stage, timeout: true }), 6000);
    socket.once("secureConnect", () => { stage = "mqtt_connack"; socket.write(cameraMqttConnect(clientId, auth.username, auth.password)); });
    socket.on("data", data => { chunks.push(data); bytes += data.length; if (bytes >= 4) finish({ stage, ...parseCameraConnack(Buffer.concat(chunks, bytes).subarray(0, 4)) }); });
    socket.once("error", () => finish({ valid: false, stage, connectionError: true }));
    socket.once("close", () => finish({ valid: false, stage, closed: true }));
  });
  return { fixedClient: await probe("com.roborock.smart:mbrriot"), uniqueClient: await probe(`roboclaw-${randomUUID()}`) };
}
