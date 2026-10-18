import { test } from "node:test";
import assert from "node:assert/strict";
import { safeReason } from "./secret-store.ts";

test("startup failure reasons keep errno codes and fixed labels, never paths or secrets", () => {
  const fsError = Object.assign(new Error("EACCES: permission denied, open '/data/camera.json'"), { code: "EACCES" });
  assert.equal(safeReason(fsError), "EACCES");
  assert.equal(safeReason(new Error("Unsafe state owner.")), "Unsafe state owner.");
  assert.equal(safeReason(new SyntaxError("Unexpected token s in JSON at position 3: secret123")), "SyntaxError");
  assert.equal(safeReason("raw string"), "unknown");
});
