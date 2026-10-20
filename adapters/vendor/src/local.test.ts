import assert from "node:assert/strict";
import { createCipheriv, createHash } from "node:crypto";
import { test } from "node:test";
import { localProtocolCodec } from "./local.ts";

const KEY = "b8Hj5mFk3QzT7rLp";
const ACK_NONCE = 485592656;

test("CRC32 matches the standard check vector", () => {
  assert.equal(localProtocolCodec.crc32(Buffer.from("123456789")), 0xcbf43926);
  assert.equal(localProtocolCodec.crc32(Buffer.alloc(0)), 0);
});

test("L01 matches upstream cryptoEngine.test.ts official vector", () => {
  // Fixture from copystring/ioBroker.roborock, under the MIT notice in local.ts.
  const context = {
    version: "L01" as const, localKey: KEY, timestamp: 1753606905, sequence: 1,
    random: 304251, connectNonce: 893563, ackNonce: ACK_NONCE,
  };
  const plain = Buffer.from('{"dps":{"101":"{\\"id\\":1806,\\"method\\":\\"get_prop\\",\\"params\\":[\\"get_status\\"]}"},"t":1753606905}');
  const encrypted = Buffer.from([
    253,96,200,218,202,28,202,230,127,96,119,71,123,250,157,55,24,154,56,215,91,60,74,144,124,36,53,211,193,70,238,132,216,249,149,151,
    227,225,87,26,1,89,97,206,170,77,100,188,54,149,250,224,36,195,65,103,55,215,113,80,52,29,226,156,173,47,149,191,175,83,35,88,241,
    43,191,248,159,20,15,239,91,30,226,132,195,171,254,59,131,165,119,145,10,114,5,109,171,77,90,117,177,130,209,160,203,161,69,227,
    228,80,243,146,116,67,
  ]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), encrypted);
  assert.deepEqual(localProtocolCodec.decryptPayload(encrypted, context), plain);
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, ackNonce: ACK_NONCE + 1 }));
  assert.throws(() => localProtocolCodec.decryptPayload(encrypted, { ...context, connectNonce: context.connectNonce + 1 }));
});

test("V1 follows the independent upstream MD5 timestamp permutation and AES-ECB specification", () => {
  const context = {
    version: "1.0" as const, localKey: KEY, timestamp: 0x12345678,
    sequence: 1, random: 2, connectNonce: 3, ackNonce: 4,
  };
  const key = createHash("md5").update("67482315" + KEY + "TXdfu$jyZ#TZHsg4").digest();
  const cipher = createCipheriv("aes-128-ecb", key, null);
  const plain = Buffer.from("read-only status");
  const expected = Buffer.concat([cipher.update(plain), cipher.final()]);
  assert.deepEqual(localProtocolCodec.encryptPayload(plain, context), expected);
  assert.deepEqual(localProtocolCodec.decryptPayload(expected, context), plain);
});
