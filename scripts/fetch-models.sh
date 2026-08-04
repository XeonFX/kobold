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

def handle_extras(model: dict, base_url: str) -> tuple[int, int]:
    extra_ok = extra_warn = 0
    for extra_spec in model.get("extra_files") or []:
        if isinstance(extra_spec, str):
            extra = extra_spec
            extra_want = ""
        else:
            extra = extra_spec["file"]
            extra_want = (extra_spec.get("sha256") or "").strip()
        ex_url = base_url.rsplit("/", 1)[0] + "/" + extra
        ex_target = dest / extra

        if ex_target.exists() and mode != "force":
            if not extra_want or sha256(ex_target) == extra_want:
                detail = "checksum verified" if extra_want else "no checksum"
                print(f"  {C('OK', '32')} companion {extra} ({detail})")
                extra_ok += 1
                continue
            print(f"  {C('CORRUPT', '31')} companion checksum mismatch: {extra}")
            extra_warn += 1
            if mode == "verify":
                continue

        if mode == "verify":
            print(f"  {C('MISSING', '31')} companion {extra}")
            extra_warn += 1
            continue

        print(f"  fetching companion {extra}")
        ex_tmp = ex_target.with_suffix(ex_target.suffix + ".part")
        try:
            urllib.request.urlretrieve(ex_url, ex_tmp)
        except Exception as error:
            print(f"    {C('WARN', '33')} {error}")
            ex_tmp.unlink(missing_ok=True)
            extra_warn += 1
            continue
        if extra_want and sha256(ex_tmp) != extra_want:
            print(f"    {C('WARN', '33')} companion checksum mismatch: {extra}")
            ex_tmp.unlink(missing_ok=True)
            extra_warn += 1
            continue
        ex_tmp.replace(ex_target)
        extra_ok += 1
    return extra_ok, extra_warn

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
            extra_ok, extra_warn = handle_extras(m, url)
            ok += extra_ok
            warn_count += extra_warn
            continue

    if mode == "verify":
        print(f"  {C('MISSING', '31')}")
        warn_count += 1
        extra_ok, extra_warn = handle_extras(m, url)
        ok += extra_ok
        warn_count += extra_warn
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
        extra_ok, extra_warn = handle_extras(m, url)
        ok += extra_ok
        warn_count += extra_warn
        continue

    if want:
        got = sha256(tmp)
        if got != want:
            print(f"  {C('FAILED', '31')} checksum mismatch after download — not installing")
            print(f"    want {want}\n    got  {got}")
            tmp.unlink(missing_ok=True)
            warn_count += 1
            extra_ok, extra_warn = handle_extras(m, url)
            ok += extra_ok
            warn_count += extra_warn
            continue

    tmp.rename(target)
    print(f"  {C('OK', '32')} installed -> {target}")
    ok += 1

    extra_ok, extra_warn = handle_extras(m, url)
    ok += extra_ok
    warn_count += extra_warn

print(f"\n{ok} ok, {warn_count} need attention")
sys.exit(1 if warn_count and mode == "verify" else 0)
PY

log "models in $DEST:"
du -sh "$DEST"/* 2>/dev/null | sed 's/^/  /' || echo "  (empty)"
