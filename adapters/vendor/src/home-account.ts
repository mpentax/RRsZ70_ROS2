/** Read-only, selected-home Roborock account access. Never log/return authorization. */
import { hawkHeaders, roomsFromHome, type AccountRoom, type HomeAuthorization } from "./cloud.ts";

export function validateHomeAuthorization(value: unknown): HomeAuthorization {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("Invalid private home authorization.");
  const home = value as Record<string, unknown>;
  if (Object.keys(home).sort().join(",") !== "deviceId,hawkKey,homeId,realApi,rooms,secret,user" ||
    typeof home.deviceId !== "string" || !/^[\x20-\x7e]{1,128}$/.test(home.deviceId) ||
    typeof home.homeId !== "number" || !Number.isSafeInteger(home.homeId) || home.homeId <= 0 ||
    typeof home.realApi !== "string" ||
    ![home.user, home.secret, home.hawkKey].every(field => typeof field === "string" &&
      field.length > 0 && field.length <= 4096 && /^[\x20-\x7e]+$/.test(field)) ||
    !Array.isArray(home.rooms) || home.rooms.length > 128) throw new Error("Invalid private home authorization.");
  let url: URL;
  try { url = new URL(home.realApi); } catch { throw new Error("Invalid private home authorization."); }
  if (url.origin !== home.realApi || url.protocol !== "https:" || !url.hostname.endsWith(".roborock.com") ||
    url.username || url.password || url.port || url.pathname !== "/" || url.search || url.hash ||
    !/^(?:[a-z0-9](?:[a-z0-9-]*[a-z0-9])?\.)+roborock\.com$/.test(url.hostname)) {
    throw new Error("Invalid private home authorization.");
  }
  const rooms = new Set<string>();
  for (const room of home.rooms) {
    if (!room || typeof room !== "object" || Array.isArray(room) || Object.keys(room).sort().join(",") !== "id,name" ||
      typeof room.id !== "string" || !/^[0-9]{1,24}$/.test(room.id) ||
      typeof room.name !== "string" || room.name.length > 120 || /[\u0000-\u001f\u007f]/.test(room.name) ||
      rooms.has(room.id)) throw new Error("Invalid private home authorization.");
    rooms.add(room.id);
  }
  return { deviceId: home.deviceId, homeId: home.homeId, realApi: home.realApi,
    user: home.user as string, secret: home.secret as string, hawkKey: home.hawkKey as string,
    rooms: home.rooms as AccountRoom[] };
}

export function selectedRooms(home: HomeAuthorization): { source: string; rooms: AccountRoom[] } {
  return { source: "roborock-account-home", rooms: home.rooms.map(room => ({ id: room.id, name: room.name })) };
}

export async function refreshRooms(home: HomeAuthorization): Promise<AccountRoom[]> {
  home = validateHomeAuthorization(home);
  const url = new URL(`/v3/user/homes/${home.homeId}`, home.realApi);
  const response = await fetch(url, {
    method: "GET", redirect: "error", signal: AbortSignal.timeout(12_000),
    headers: hawkHeaders(url, home.user, home.secret, home.hawkKey),
  });
  if (!response.ok) { await response.body?.cancel(); throw new Error("Home room refresh failed; account reauthentication may be required."); }
  const declared = Number(response.headers.get("content-length"));
  if (response.headers.has("content-length") && (declared > 2 * 1024 * 1024 || !Number.isSafeInteger(declared))) {
    await response.body?.cancel(); throw new Error("Home room response exceeded the safe size limit.");
  }
  if (!response.body) throw new Error("Home room response was empty.");
  let size = 0;
  const chunks: Uint8Array[] = [];
  for await (const chunk of response.body) {
    size += chunk.byteLength;
    if (size > 2 * 1024 * 1024) { await response.body.cancel().catch(() => {}); throw new Error("Home room response exceeded the safe size limit."); }
    chunks.push(chunk);
  }
  try { return roomsFromHome(JSON.parse(Buffer.concat(chunks, size).toString("utf8"))); }
  catch { throw new Error("Invalid home room response."); }
}
