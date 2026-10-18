import { randomUUID } from 'node:crypto';
import type { ServerResponse } from 'node:http';

/** Live dispatch evidence only. No vendor payloads, secrets, or history replay. */
export type CommandEvent = {
  sessionId: string; sequence: number; commandId: string; attemptId: string; attemptIndex: number;
  phase: 'accepted' | 'dispatched' | 'replied' | 'finished'; operation: string;
  requestedAt: number; attemptedAt: number | null; finishedAt: number | null;
  writeTimeKnown: false; dispatch: 'not_sent' | 'sent' | 'unknown';
  response: 'none' | 'received' | 'vendor_error' | 'transport_error';
  errorCode: string; vendorCode?: number;
  params?: { velocity: number; omega: number; duration: number };
};
export class CommandEvents {
  readonly sessionId = randomUUID();
  private sequence = 0;
  private clients = new Set<ServerResponse>();
  subscribe(res: ServerResponse) {
    if (this.clients.size >= 8) { res.writeHead(503); res.end(); return; }
    res.writeHead(200, { 'Content-Type': 'application/x-ndjson', 'Cache-Control': 'no-store' });
    res.flushHeaders();
    this.clients.add(res);
    const heartbeat = setInterval(() => {
      if (res.destroyed || res.writableLength > 64 * 1024) res.destroy();
      else res.write('\n');
    }, 1000);
    heartbeat.unref();
    res.once('close', () => { clearInterval(heartbeat); this.clients.delete(res); });
  }
  emit(event: Omit<CommandEvent, 'sessionId' | 'sequence' | 'writeTimeKnown'>) {
    const row: CommandEvent = { ...event, sessionId: this.sessionId, sequence: ++this.sequence, writeTimeKnown: false };
    const line = JSON.stringify(row) + '\n';
    for (const res of this.clients) {
      // Never hold up a motor command for an observer. Reconnect exposes a sequence gap.
      if (res.destroyed || res.writableLength > 64 * 1024) { this.clients.delete(res); res.destroy(); }
      else res.write(line);
    }
  }
  close() { for (const res of this.clients) res.end(); this.clients.clear(); }
}
export function pulseParams(method: string, params: unknown): CommandEvent['params'] {
  if (method !== 'app_rc_move' || !Array.isArray(params) || params.length !== 1) return;
  const p = params[0];
  if (!p || typeof p !== 'object') return;
  const { velocity, omega, duration } = p as Record<string, unknown>;
  if ([velocity, omega, duration].every(n => typeof n === 'number' && Number.isFinite(n)) &&
      Math.abs(velocity as number) <= 10 && Math.abs(omega as number) <= 10 &&
      (duration as number) > 0 && (duration as number) <= 120000)
    return { velocity: velocity as number, omega: omega as number, duration: duration as number };
}
