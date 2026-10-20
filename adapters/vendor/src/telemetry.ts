/** Numeric transport sanitization only. ROS owns polling and semantic telemetry. */
export type NumericTelemetry = number | boolean | null | NumericTelemetry[] | { [key: string]: NumericTelemetry };
export function numericTelemetry(value: unknown, depth = 0): NumericTelemetry {
  if (depth > 5) return null;
  if (typeof value === "number") return Number.isFinite(value) ? value : null;
  if (typeof value === "boolean") return value;
  if (Array.isArray(value)) return value.slice(0, 64).map((item) => numericTelemetry(item, depth + 1));
  if (!value || typeof value !== "object") return null;
  const result: Record<string, NumericTelemetry> = {};
  for (const [key, item] of Object.entries(value).slice(0, 64)) {
    if (!/^[a-zA-Z][a-zA-Z0-9_]{0,63}$/.test(key) ||
      /key|token|secret|password|ssid|serial|duid|(^|_)(mac|ip|url)($|_)/i.test(key)) continue;
    const filtered = numericTelemetry(item, depth + 1);
    if (filtered !== null) result[key] = filtered;
  }
  return result;
}
