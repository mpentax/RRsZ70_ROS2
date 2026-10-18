import test from "node:test";
import assert from "node:assert/strict";
import * as diagnostic from "./camera-broker-diagnostic.ts";
import { EventEmitter } from "node:events";
import type { connect, TLSSocket } from "node:tls";

test("broker diagnostic sends CONNECT only, reads fragmented CONNACK and closes both sessions", async () => {
  const packets: Buffer[] = []; let closed = 0;
  const connector = (() => {
    const socket = new EventEmitter() as EventEmitter & { write(data: Buffer): void; destroy(): void };
    socket.write = data => { packets.push(data); socket.emit("data", Buffer.from([0x20, 2])); socket.emit("data", Buffer.from([1, 0])); };
    socket.destroy = () => { closed++; };
    queueMicrotask(() => socket.emit("secureConnect"));
    return socket as unknown as TLSSocket;
  }) as typeof connect;
  const result = await diagnostic.cameraBrokerDiagnostic({ user: "user", secret: "secret", key: "account", mqttUrl: "ssl://mqtt-eu.roborock.com:8883" },
    { duid: "test-z70", localKey: "0123456789abcdef", name: "test", model: "a179", protocolVersion: "1.0" }, connector);
  assert.equal(packets.length, 2); assert.equal(closed, 2);
  for (const packet of packets) { assert.equal(packet[0], 0x10); assert.equal(packet[1], packet.length - 2); assert.equal(packet[9], 0xc2); }
  assert.equal(result.fixedClient.accepted, true); assert.equal(result.uniqueClient.accepted, true);
  assert.equal(JSON.stringify(result).includes("secret"), false);
});

test("CONNACK acceptance distinguishes successful session-present from login rejection", () => {
  assert.deepEqual(diagnostic.parseCameraConnack(Buffer.from([0x20, 2, 1, 0])), { valid: true, accepted: true, sessionPresent: true, returnCode: 0 });
  assert.deepEqual(diagnostic.parseCameraConnack(Buffer.from([0x20, 2, 0, 5])), { valid: true, accepted: false, sessionPresent: false, returnCode: 5 });
  assert.deepEqual(diagnostic.parseCameraConnack(Buffer.from([0x20, 2, 0, 0])), { valid: true, accepted: true, sessionPresent: false, returnCode: 0 });
  assert.deepEqual(diagnostic.parseCameraConnack(Buffer.from([0x30, 2, 0, 0])), { valid: false });
});
