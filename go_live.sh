#!/bin/bash
# Start the race server + a Cloudflare quick tunnel on this Mac, then point the Vercel site at it.
# Safe to re-run any time (e.g. after a reboot or if the tunnel dies): it reuses whatever is still running.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p data
PORT=8765

# 1) race server (caffeinate keeps the Mac awake while it runs)
if ! lsof -iTCP:$PORT -sTCP:LISTEN >/dev/null 2>&1; then
  echo "starting race server…"
  nohup caffeinate -is python3 server.py >> data/server.log 2>&1 &
  sleep 2
fi

# 2) tunnel: reuse it if it's alive, otherwise start a fresh one
URL=""
if pgrep -f "cloudflared tunnel .*localhost:$PORT" >/dev/null; then
  URL=$(grep -ao 'https://[a-z0-9-]*\.trycloudflare\.com' data/tunnel.log 2>/dev/null | head -1 || true)
  if [ -n "$URL" ] && ! curl -sf -m 8 "$URL/api/me" >/dev/null; then URL=""; pkill -f "cloudflared tunnel .*localhost:$PORT" || true; fi
fi
if [ -z "$URL" ]; then
  echo "starting cloudflare tunnel…"
  nohup cloudflared tunnel --no-autoupdate --url http://localhost:$PORT > data/tunnel.log 2>&1 &
  for _ in $(seq 60); do
    URL=$(grep -ao 'https://[a-z0-9-]*\.trycloudflare\.com' data/tunnel.log | head -1 || true)
    [ -n "$URL" ] && break; sleep 1
  done
  [ -z "$URL" ] && { echo "tunnel failed, see data/tunnel.log"; exit 1; }
  for _ in $(seq 30); do curl -sf -m 5 "$URL/api/me" >/dev/null && break; sleep 2; done
fi
echo "race server reachable at $URL"

# 3) publish the tunnel URL where the Vercel site looks it up (a secret GitHub gist), so the site
#    never needs redeploying when the tunnel changes.
echo "{\"api\": \"$URL\"}" > data/pitwall.json
if [ ! -s data/gist_id ]; then
  gh gist create -d "PITWALL live race server address" data/pitwall.json | grep -o '[0-9a-f]\{32\}' > data/gist_id
else
  gh gist edit "$(cat data/gist_id)" -f pitwall.json data/pitwall.json
fi
SRC="https://gist.githubusercontent.com/$(gh api user -q .login)/$(cat data/gist_id)/raw/pitwall.json"
echo "window.PITWALL_API_SOURCE = \"$SRC\";" > static/config.js
echo "site reads the server address from $SRC"

# 4) deploy the static site to Vercel. Only needed once (or when you change files in static/);
#    run it from a real terminal the first time, since Vercel may ask for an email/passkey check.
if [ "${1:-}" = "--deploy" ]; then
  [ -d static/.vercel ] || vercel link --yes --project "${VERCEL_PROJECT:-pitwall}" --cwd static
  vercel deploy --prod --cwd static
fi
