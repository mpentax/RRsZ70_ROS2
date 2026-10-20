/*
 * Protocol/crypto adapted from copystring/ioBroker.roborock:
 * src/lib/{localApi,cryptoEngine,messageParser}.ts (retrieved 2026-09-24).
 * Also cross-checked against Python-roborock/python-roborock:
 * roborock/protocol.py, protocols/v1_protocol.py, devices/transport/local_channel.py.
 *
 * MIT License
 * Copyright (c) 2025-2026 copystring <copystring@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
import { createCipheriv, createDecipheriv, createHash } from "node:crypto";

type Version = "1.0" | "L01";
type CryptoContext = {
  version: Version;
  localKey: string;
  timestamp: number;
  sequence: number;
  random: number;
  connectNonce: number;
  ackNonce: number;
};

const SALT = "TXdfu$jyZ#TZHsg4";

class LocalProtocolError extends Error {}

function crc32(data: Buffer): number {
  let crc = 0xffffffff;
  for (const byte of data) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) {
      crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
  }
  return (crc ^ 0xffffffff) >>> 0;
}

function words(...values: number[]): Buffer {
  const result = Buffer.alloc(values.length * 4);
  values.forEach((value, index) => result.writeUInt32BE(value, index * 4));
  return result;
}

function keyMaterial(context: CryptoContext): string {
  const hex = context.timestamp.toString(16).padStart(8, "0");
  return [5, 6, 3, 7, 1, 2, 0, 4].map((index) => hex[index]).join("") + context.localKey + SALT;
}

function gcmParameters(context: CryptoContext): { key: Buffer; iv: Buffer; aad: Buffer } {
  return {
    key: createHash("sha256").update(keyMaterial(context)).digest(),
    iv: createHash("sha256").update(words(context.sequence, context.random, context.timestamp)).digest().subarray(0, 12),
    aad: words(context.sequence, context.connectNonce, context.ackNonce, context.random, context.timestamp),
  };
}

function encryptPayload(payload: Buffer, context: CryptoContext): Buffer {
  if (context.version === "1.0") {
    const key = createHash("md5").update(keyMaterial(context)).digest();
    const cipher = createCipheriv("aes-128-ecb", key, null);
    return Buffer.concat([cipher.update(payload), cipher.final()]);
  }
  const { key, iv, aad } = gcmParameters(context);
  const cipher = createCipheriv("aes-256-gcm", key, iv);
  cipher.setAAD(aad);
  return Buffer.concat([cipher.update(payload), cipher.final(), cipher.getAuthTag()]);
}

function decryptPayload(payload: Buffer, context: CryptoContext): Buffer {
  if (context.version === "1.0") {
    if (!payload.length || payload.length % 16 !== 0) {
      throw new LocalProtocolError("Invalid V1 encrypted payload length");
    }
    const key = createHash("md5").update(keyMaterial(context)).digest();
    const decipher = createDecipheriv("aes-128-ecb", key, null);
    return Buffer.concat([decipher.update(payload), decipher.final()]);
  }
  if (payload.length < 16) throw new LocalProtocolError("Invalid L01 encrypted payload length");
  const { key, iv, aad } = gcmParameters(context);
  const decipher = createDecipheriv("aes-256-gcm", key, iv);
  decipher.setAAD(aad);
  decipher.setAuthTag(payload.subarray(-16));
  return Buffer.concat([decipher.update(payload.subarray(0, -16)), decipher.final()]);
}

export const localProtocolCodec = Object.freeze({ crc32, encryptPayload, decryptPayload });
