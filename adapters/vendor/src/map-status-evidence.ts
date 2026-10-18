import type { Observation } from "./telemetry.ts";

/** Read-only, bounded get_status fields. Not a verified active-map ID or pose. */
export interface MapStatusEvidence {
  mapStatus: number | null;
  mapStatusLowBits: number | null;
  isLocating: boolean | null;
  // Hermes #8421 calculates map_status >> 2, but no equivalence with V1 mapIndex is established.
  pluginMapIdCandidate: number | null;
  candidateUsable: boolean;
  interpretation: "unverified";
}

function object(value: unknown): value is Record<string, unknown> {
  return value !== null && typeof value === "object" && !Array.isArray(value);
}

export function parseMapStatusEvidence(response: unknown): MapStatusEvidence | null {
  const status: unknown = Array.isArray(response) && response.length === 1 ? response[0] : response;
  if (!object(status)) return null;
  const raw = status.map_status;
  const mapStatus = typeof raw === "number" && Number.isSafeInteger(raw) && raw >= -2147483648 && raw <= 2147483647
    ? raw : null;
  const locating = status.is_locating;
  const isLocating = locating === true || locating === 1 ? true : locating === false || locating === 0 ? false : null;
  if (mapStatus === null && isLocating === null) return null;
  // In Hermes v96 #8421: parseInt(map_status) % 4 indexes a UI enum;
  // map_status >> 2 is the plugin's candidate ID. A -1 shifted value,
  // or bit 7 set, forces -1; isLocating also forces -1. The V1 header's
  // mapIndex has no proven shared identity with this candidate.
  const shifted = mapStatus === null ? null : mapStatus >> 2;
  const candidate = mapStatus === null || isLocating === null ? null
    : isLocating || shifted === -1 || ((mapStatus >> 7) & 1) === 1 ? -1 : shifted;
  const candidateUsable = candidate !== null && candidate >= 0 && isLocating === false;
  return { mapStatus, mapStatusLowBits: mapStatus === null ? null : mapStatus & 3,
    isLocating, pluginMapIdCandidate: candidate, candidateUsable, interpretation: "unverified" };
}

export function unavailableMapStatusEvidence(error: string | null = null): Observation<MapStatusEvidence> {
  return { value: null, observedAt: null, error };
}
