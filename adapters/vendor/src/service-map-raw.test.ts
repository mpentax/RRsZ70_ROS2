import test from 'node:test';
import assert from 'node:assert/strict';
import { createService, type ServiceStore } from './service.ts';
import { loadServiceConfig } from './service-config.ts';
import { syntheticV1Image } from './map-raster-fixture.ts';
import { mapMetadataFromRead } from './map-cache-status.ts';
import { createHash } from 'node:crypto';

test('authenticated raw map reads preserve exact verified bytes and selected-device binding', async t => {
  const token = 'x'.repeat(43), raw = syntheticV1Image();
  const device = { duid: 'robot1', host: '127.0.0.1', localKey: '1234567890abcdef', name: 'test', model: 'a179', protocolVersion: 'L01' };
  const metadata = mapMetadataFromRead(device.duid, { bytes: raw.length, sha256: createHash('sha256').update(raw).digest('hex'), mapIndex: 2, mapSequence: 3 }, 12345);
  let selected = true;
  const camera = { account: { user: 'u', secret: 's', key: 'k', mqttUrl: 'ssl://mqtt-eu.roborock.com:8883' }, pin: '1234' };
  const home = { deviceId: device.duid, homeId: 1, realApi: 'https://api-eu.roborock.com', user: 'u', secret: 's', hawkKey: 'k', rooms: [] };
  const store: ServiceStore = { async initialize() {}, async serviceToken() { return token; }, async loadPairing() { return { device, camera, home }; }, async savePairing() {},
    async loadMapRasterSource() { return { raw, metadata: { ...metadata, deviceId: selected ? device.duid : 'other' } }; } };
  const config = { ...loadServiceConfig(), port: 0 };
  const service = await createService({ store, config, robotFactory: () => ({ connection: { state: 'connected', connectedAt: 1 }, close() {}, async request() { throw new Error('Raw map must not contact robot'); } }) });
  await service.start(); t.after(() => service.stop());
  const address = service.server.address(); assert.ok(address && typeof address === 'object');
  const origin = `http://127.0.0.1:${address.port}`; config.allowedHosts.push(`127.0.0.1:${address.port}`); config.publicOrigin = origin;
  assert.equal((await fetch(origin + '/api/map/cache/raw')).status, 401);
  const response = await fetch(origin + '/api/map/cache/raw', { headers: { Authorization: `Bearer ${token}` } });
  assert.equal(response.status, 200);
  assert.equal(response.headers.get('content-type'), 'application/octet-stream');
  assert.equal(response.headers.get('x-map-saved-at'), '12345');
  assert.deepEqual(Buffer.from(await response.arrayBuffer()), raw);
  selected = false;
  assert.equal((await fetch(origin + '/api/map/cache/raw', { headers: { Authorization: `Bearer ${token}` } })).status, 404);
});
