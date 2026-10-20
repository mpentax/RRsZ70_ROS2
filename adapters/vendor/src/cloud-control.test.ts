import { test } from "node:test";
import assert from "node:assert/strict";
import { EventEmitter } from "node:events";
import { localProtocolCodec } from "./local.ts";
import { CloudControlClient, encodeCloudRpc, decodeCloudRpc, mqttOptions, type CloudMqttPeer } from "./cloud-control.ts";

const credentials = {
  broker: "mqtts://mqtt-eu.roborock.com:8883", user: "user", secret: "secret",
  accountKey: "account", deviceId: "test-z70", localKey: "0123456789abcdef",
};

function reply(id: number, result: unknown): Buffer {
  const timestamp = 1753606905;
  const payload = localProtocolCodec.encryptPayload(Buffer.from(JSON.stringify({
    dps: { "102": JSON.stringify({ id, result }) },
  })), { version: "1.0", localKey: credentials.localKey, timestamp, sequence: id, random: timestamp, ackNonce: 0, connectNonce: 0 });
  const frame = Buffer.alloc(23 + payload.length);
  frame.write("1.0"); frame.writeUInt32BE(id, 3); frame.writeUInt32BE(timestamp, 7);
  frame.writeUInt32BE(timestamp, 11); frame.writeUInt16BE(102, 15); frame.writeUInt16BE(payload.length, 17);
  payload.copy(frame, 19); frame.writeUInt32BE(localProtocolCodec.crc32(frame.subarray(0, -4)), frame.length - 4);
  return frame;
}

class FakeBroker extends EventEmitter implements CloudMqttPeer {
  connected = true;
  frames: Buffer[] = [];
  respond = true;
  subscribe(_topic: string, _options: { qos: 0 }, callback: (error?: Error | null, granted?: { qos: number }[]) => void) {
    callback(null, [{ qos: 0 }]);
  }
  publish(_topic: string, frame: Buffer, _options: { qos: 0; retain: false }, callback: (error?: Error) => void) {
    this.frames.push(frame); callback();
    if (this.respond) queueMicrotask(() => this.emit("message", mqttOptions(credentials).incoming, reply(frame.readUInt32BE(3), ["ok"])));
  }
  end() { this.connected = false; }
}

test('cloud priority stop has reserved capacity while ordinary replies hang',async()=>{
  const broker=new FakeBroker();const client=new CloudControlClient(credentials,async()=>broker);
  try {
    await client.rpc('get_status');broker.respond=false;
    const pending=Array.from({length:4},()=>client.rpc('get_wait',[],10000).catch(e=>e));
    await new Promise(resolve=>setImmediate(resolve));
    await assert.rejects(client.rpc('app_grip'),/capacity/);
    broker.respond=true;
    assert.deepEqual(await client.rpc('app_stop_grasp',[],1000),['ok']);
    assert.equal(broker.frames.length,6);
    client.close();await Promise.all(pending);
  } finally {client.close();}
});

test("MQTT frames reuse V1 crypto, correct protocol IDs and CRC without TCP prefix", () => {
  const frame = encodeCloudRpc(credentials.localKey, 12, "get_status", [], 1753606905);
  assert.equal(frame.toString("ascii", 0, 3), "1.0");
  assert.equal(frame.readUInt16BE(15), 101);
  assert.equal(frame.readUInt16BE(17) + 23, frame.length);
  assert.equal(frame.readUInt32BE(frame.length - 4), localProtocolCodec.crc32(frame.subarray(0, -4)));
  assert.deepEqual(decodeCloudRpc(credentials.localKey, reply(12, ["ok"])), { id: 12, result: ["ok"] });
  const corrupted = reply(12, ["ok"]); corrupted[20] ^= 1;
  assert.throws(() => decodeCloudRpc(credentials.localKey, corrupted), /Invalid cloud/);
});

test("binary V1 reads include only the supplied per-request security nonce", () => {
  const nonce = Buffer.from("0123456789abcdef");
  const frame = encodeCloudRpc(credentials.localKey, 12345, "get_map_v1", [], 1753606905,
    { endpoint: "abcdefgh", nonce });
  const plain = localProtocolCodec.decryptPayload(frame.subarray(19, -4), {
    version: "1.0", localKey: credentials.localKey, timestamp: 1753606905, sequence: 12345,
    random: 1753606905, ackNonce: 0, connectNonce: 0,
  });
  const outer = JSON.parse(plain.toString("utf8")) as { dps: { "101": string } };
  assert.deepEqual(JSON.parse(outer.dps["101"]), {
    id: 12345, method: "get_map_v1", params: [],
    security: { endpoint: "abcdefgh", nonce: nonce.toString("hex") },
  });
  assert.throws(() => encodeCloudRpc(credentials.localKey, 1, "get_map_v1", [], 1,
    { endpoint: "bad", nonce }), /security/);
});

test("cloud broker validation accepts the trusted regional numeric shard without widening the domain", () => {
  assert.equal(mqttOptions({ ...credentials, broker: "mqtts://mqtt-eu-1.roborock.com:8883" }).incoming,
    mqttOptions(credentials).incoming);
  for (const broker of ["mqtts://mqtt-eu-1.attacker.invalid:8883", "mqtts://mqtt-eu-1.roborock.com.attacker.invalid:8883",
    "mqtts://mqtt-eu-1-evil.roborock.com:8883"]) {
    assert.throws(() => mqttOptions({ ...credentials, broker }), /credentials/);
  }
});

test("cloud broker validation rejects arbitrary hosts and insecure transport", () => {
  for (const broker of ["mqtt://mqtt.roborock.com:1883", "mqtts://attacker.invalid:8883"]) {
    assert.throws(() => mqttOptions({ ...credentials, broker }), /credentials/);
  }
});

function trackedClient() {
  const brokers: FakeBroker[] = [];
  const client = new CloudControlClient(credentials, async () => { const broker = new FakeBroker(); brokers.push(broker); return broker; });
  return { brokers, client };
}

test("generic cloud writes connect on demand and next explicit call recovers after timeout without replay", async () => {
  const { brokers, client } = trackedClient();
  try {
    assert.deepEqual(await client.rpc("app_charge", {}), ["ok"]);
    brokers[0].respond = false;
    await assert.rejects(client.rpc("app_unknown", {}, 5), error => !!error && typeof error === "object" && "dispatch" in error && error.dispatch === "unknown");
    assert.deepEqual(await client.rpc("app_next", {}), ["ok"]);
    assert.equal(brokers.length, 2); assert.equal(brokers[0].frames.length, 2); assert.equal(brokers[1].frames.length, 1);
  } finally { client.close(); }
});

test("delayed events from old cloud client cannot terminate a new explicit request", async () => {
  const { brokers, client } = trackedClient();
  try {
    await client.rpc("first"); brokers[0].respond = false;
    await assert.rejects(client.rpc("timeout", [], 5));
    assert.deepEqual(await client.rpc("next"), ["ok"]);
    brokers[0].emit("close"); brokers[0].emit("error", new Error("late"));
    assert.deepEqual(await client.rpc("still_connected"), ["ok"]);
    assert.equal(brokers.length, 2);
  } finally { client.close(); }
});
