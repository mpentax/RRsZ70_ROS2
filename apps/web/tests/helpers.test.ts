import assert from "node:assert/strict";
import { test } from "node:test";
import { ageLabel, batteryPercent, FRAME_MAX_AGE_MS, isFresh, pollDelay,
  takeFragmentToken, timestamp } from "../src/helpers.ts";

test("frame freshness has a strict 30-second boundary and rejects missing or future observations", () => {
  const now = 1_000_000;
  assert.equal(isFresh(now - 30_000, now, FRAME_MAX_AGE_MS), true);
  assert.equal(isFresh(now - 30_001, now, FRAME_MAX_AGE_MS), false);
  assert.equal(isFresh(now + 1, now, FRAME_MAX_AGE_MS), false);
  assert.equal(isFresh(null, now, FRAME_MAX_AGE_MS), false);
  assert.equal(isFresh("not a timestamp", now, FRAME_MAX_AGE_MS), false);
  assert.equal(timestamp("2026-09-24T12:00:00Z"), Date.parse("2026-09-24T12:00:00Z"));
  assert.equal(timestamp(Infinity), null);
});

test("battery readings are not invented or clamped into plausible values", () => {
  assert.equal(batteryPercent(0), 0);
  assert.equal(batteryPercent(100), 100);
  for (const invalid of [null, undefined, "42", -1, 101, NaN]) {
    assert.equal(batteryPercent(invalid), null);
  }
});

test("fragment token is removed before returning, without moving it into a query", () => {
  let removed = false;
  const token = takeFragmentToken({ hash: "#token=a%2Bb%2Fsecret", pathname: "/", search: "?view=panel" }, {
    replaceState(data, unused, url) {
      assert.equal(data, null);
      assert.equal(unused, "");
      assert.equal(url, "/?view=panel");
      removed = true;
    },
  });
  assert.equal(removed, true);
  assert.equal(token, "a+b/secret");
  assert.equal(takeFragmentToken({ hash: "#telemetry", pathname: "/", search: "" }, {
    replaceState() { assert.fail("Non-token fragments should not change."); },
  }), null);
});

test("an empty token fragment is stripped too", () => {
  let removed = false;
  assert.equal(takeFragmentToken({ hash: "#token=", pathname: "/", search: "" }, {
    replaceState() { removed = true; },
  }), null);
  assert.equal(removed, true);
});

test("polling uses 3 seconds normally and caps exponential failure backoff at 30 seconds", () => {
  assert.deepEqual([0, 1, 2, 3, 4, 5, 100].map(pollDelay), [3_000, 6_000, 12_000, 24_000, 30_000, 30_000, 30_000]);
  assert.equal(ageLabel(null, 1000), "Not observed");
  assert.equal(ageLabel(0, 16_000), "16s ago");
});
