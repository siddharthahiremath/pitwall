#!/bin/sh
# Download the full race recording (telemetry, events, every script version, stats) from the Railway server
# into data/export/<timestamp>/. Needs data/EXPORT_TOKEN.local.txt (same value as PITWALL_EXPORT_TOKEN on Railway).
set -e
cd "$(dirname "$0")"
API=$(sed -n 's/.*PITWALL_API *= *"\([^"]*\)".*/\1/p' static/config.js)
OUT="data/export/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
curl -sf -H "Authorization: Bearer $(cat data/EXPORT_TOKEN.local.txt)" "$API/api/admin/export" | tar xf - -C "$OUT"
du -sh "$OUT"; echo "saved to $OUT"
