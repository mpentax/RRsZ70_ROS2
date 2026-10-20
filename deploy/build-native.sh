#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

GO2RTC_BIN="$ROOT/.local/bin/go2rtc"
if [[ ! -x "$GO2RTC_BIN" ]]; then
  echo "Building native darwin/arm64 go2rtc binary..."
  mkdir -p "$ROOT/.local/bin"
  WORK="$(mktemp -d "${TMPDIR:-/tmp}/go2rtc-build.XXXXXX")"
  trap 'rm -rf "$WORK"' EXIT
  curl -fsSL "https://codeload.github.com/AlexxIT/go2rtc/tar.gz/b5948cfb25404cc5cb37b166ecaa2dca20b11d4b" -o "$WORK/source.tar.gz"
  echo "78aa79bcedec8f155e4060a379613979b0b3ee48ff62ee5164bafc0ac6532386  $WORK/source.tar.gz" | shasum -a 256 -c -
  tar -xzf "$WORK/source.tar.gz" --strip-components=1 -C "$WORK"
  (
    cd "$WORK"
    go mod download
    go run "$ROOT/deploy/patch-go2rtc.go"
    go run "$ROOT/deploy/patch-go2rtc-diagnostics.go"
    CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o "$GO2RTC_BIN" .
  )
fi

if [[ ! -s "$ROOT/dist/web/index.html" ]] || [[ -n "$(find "$ROOT/apps/web" "$ROOT/packages/http-contracts" -newer "$ROOT/dist/web/index.html" 2>/dev/null)" ]]; then
  npm run build
fi

colcon build --symlink-install --base-paths src
