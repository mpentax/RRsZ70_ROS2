import { z } from "zod/v4";

/** Generic vendor RPC. Parameters are deliberately not an action allowlist. */
export const rpcSchema = {
  method: z.string().regex(/^[A-Za-z_][A-Za-z0-9_]{0,127}$/),
  params: z.unknown(),
  transport: z.enum(["local", "cloud"]).optional(),
  timeoutMs: z.number().int().positive().max(30000).optional(),
  takeover: z.boolean().optional(),
};
