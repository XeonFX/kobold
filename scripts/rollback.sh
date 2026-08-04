#!/usr/bin/env bash
# Roll back to the previously deployed commit. Runs ON the robot.
#
#   ./scripts/rollback.sh
#
# This works because deploy.sh never prunes aggressively — the previous images
# are still in the local Docker cache, so rollback is seconds rather than a
# 2 GB re-download over WiFi. Never run `docker system prune -a` on the robot.
set -euo pipefail

ROOT="${KOBOLD_ROOT:-/opt/kobold}"
DEPLOY_STATE=/data/deploy
COMPOSE="docker compose -f $ROOT/docker/compose.yaml --env-file $ROOT/docker/versions.env"
PROFILES="${KOBOLD_PROFILES:-}"

log()  { printf '\033[36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[!]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

previous=$(cat "$DEPLOY_STATE/previous" 2>/dev/null || echo "")
current=$(cat "$DEPLOY_STATE/current" 2>/dev/null || echo "unknown")
[ -n "$previous" ] || die "no previous deployment recorded — nothing to roll back to"

log "rolling back: $current -> $previous"

# Stop first. Rolling back usually means the current version is misbehaving, and
# leaving it running while the old one starts gives you two nodes fighting over
# the same serial ports and TF tree.
# shellcheck disable=SC2086
profile_args=""; for p in $PROFILES; do profile_args="$profile_args --profile $p"; done
# shellcheck disable=SC2086
$COMPOSE $profile_args down --remove-orphans || warn "compose down reported errors"

git -C "$ROOT" checkout --quiet "$previous" || die "cannot check out $previous"

# shellcheck disable=SC2086
$COMPOSE $profile_args up -d

log "restoring firmware"
"$ROOT/scripts/flash.sh" both || warn "firmware rollback reported a problem"

echo "$previous" | sudo tee "$DEPLOY_STATE/current" >/dev/null
echo "$current"  | sudo tee "$DEPLOY_STATE/rolled_back_from" >/dev/null
date -Iseconds   | sudo tee "$DEPLOY_STATE/deployed_at" >/dev/null

log "rolled back to $previous"
warn "investigate $current before deploying again"
