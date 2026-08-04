#!/usr/bin/env bash
# Download and verify models listed in models/manifest.yaml into /data/models.
#
#   ./scripts/fetch-models.sh            fetch anything missing
#   ./scripts/fetch-models.sh --verify   check what is already there, download nothing
#   ./scripts/fetch-models.sh --force    re-download everything
#
# Models live outside git and outside images on purpose: they are large, they
# change on a different cadence from code, and a 2.7 GB blob in git history is
# permanent. Swapping a model is a config change, not a deploy.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MANIFEST="$ROOT/models/manifest.yaml"
DEST="${MODELS_DIR:-/data/models}"

log()  { printf '\033[36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[!]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

MODE=fetch
case "${1:-}" in
  --verify) MODE=verify ;;
  --force)  MODE=force ;;
  "") ;;
  *) die "usage: fetch-models.sh [--verify|--force]" ;;
esac

command -v python3 >/dev/null || die "python3 required"
mkdir -p "$DEST"

# Parse the manifest with python rather than grep/awk: YAML is not a line-based
# format and a half-working parser here silently fetches the wrong thing.
python3 - "$MANIFEST" "$DEST" "$MODE" <<'PY'
import hashlib, os, subprocess, sys, urllib.request
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("PyYAML required:  pip install pyyaml")

manifest, dest, mode = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
spec = yaml.safe_load(manifest.read_text())

C = lambda s, c: f"\033[{c}m{s}\033[0m"
ok = warn_count = 0

def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()

for key, m in (spec.get("models") or {}).items():
    name, fname = m.get("name", key), m.get("file", "")
    url, want = m.get("url", ""), (m.get("sha256") or "").strip()
    target = dest / fname

    print(f"\n{C('[' + key + ']', '36')} {name}  ({m.get('size_mb', '?')} MB)")

    if not url:
        print(f"  {C('SKIP', '33')} no url in manifest — fill it in when you pick a conversion")
        warn_count += 1
        continue

    need = mode == "force" or not target.exists()

    if target.exists() and not need:
        if want:
            got = sha256(target)
            if got == want:
                print(f"  {C('OK', '32')} present, checksum verified")
                ok += 1
            else:
                print(f"  {C('CORRUPT', '31')} sha256 mismatch")
                print(f"    want {want}\n    got  {got}")
                need = mode != "verify"
                warn_count += 1
        else:
            print(f"  {C('OK', '32')} present (no checksum in manifest to verify against)")
            warn_count += 1
            ok += 1
        if not need:
            continue

    if mode == "verify":
        print(f"  {C('MISSING', '31')}")
        warn_count += 1
        continue

    print(f"  downloading {url}")
    tmp = target.with_suffix(target.suffix + ".part")
    try:
        # curl rather than urllib: resume support and a progress bar matter for
        # multi-GB files over home WiFi.
        subprocess.run(["curl", "-fL", "--progress-bar", "-C", "-", "-o", str(tmp), url], check=True)
    except subprocess.CalledProcessError:
        print(f"  {C('FAILED', '31')} download error")
        tmp.unlink(missing_ok=True)
        warn_count += 1
        continue

    if want:
        got = sha256(tmp)
        if got != want:
            print(f"  {C('FAILED', '31')} checksum mismatch after download — not installing")
            print(f"    want {want}\n    got  {got}")
            tmp.unlink(missing_ok=True)
            warn_count += 1
            continue

    tmp.rename(target)
    print(f"  {C('OK', '32')} installed -> {target}")
    ok += 1

    for extra in m.get("extra_files") or []:
        ex_url = url.rsplit("/", 1)[0] + "/" + extra
        ex_target = dest / extra
        if not ex_target.exists():
            print(f"  fetching companion {extra}")
            try:
                urllib.request.urlretrieve(ex_url, ex_target)
            except Exception as e:
                print(f"    {C('WARN', '33')} {e}")

print(f"\n{ok} ok, {warn_count} need attention")
sys.exit(1 if warn_count and mode == "verify" else 0)
PY

log "models in $DEST:"
du -sh "$DEST"/* 2>/dev/null | sed 's/^/  /' || echo "  (empty)"
