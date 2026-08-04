#!/usr/bin/env python3
"""Generate the C++ and Python protocol bindings from protocol/protocol.yaml.

    python3 tools/gen_protocol.py

Writes:
    firmware/lib/kobold_protocol/protocol_generated.h
    ros2_ws/src/kobold_bridge/kobold_bridge/protocol_generated.py

Both are committed to the repo so that neither building the firmware nor
running the bridge requires PyYAML at build time. Re-run this and commit the
result whenever protocol.yaml changes.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("PyYAML required:  pip install pyyaml")

ROOT = Path(__file__).resolve().parent.parent
SPEC = ROOT / "protocol" / "protocol.yaml"
OUT_H = ROOT / "firmware" / "lib" / "kobold_protocol" / "protocol_generated.h"
OUT_PY = (
    ROOT / "ros2_ws" / "src" / "kobold_bridge" / "kobold_bridge" / "protocol_generated.py"
)

# type name -> (C++ type, python struct code, size in bytes)
TYPES = {
    "u8": ("uint8_t", "B", 1),
    "i8": ("int8_t", "b", 1),
    "u16": ("uint16_t", "H", 2),
    "i16": ("int16_t", "h", 2),
    "u32": ("uint32_t", "I", 4),
    "i32": ("int32_t", "i", 4),
    "f32": ("float", "f", 4),
}

BANNER = "GENERATED FROM protocol/protocol.yaml -- DO NOT EDIT BY HAND."


def git_hash() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short=8", "HEAD"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        return out.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "untracked"


def camel(name: str) -> str:
    return "".join(p.capitalize() for p in name.split("_"))


def fixed_fields(msg):
    """Fields excluding a trailing variable-length `str`, plus that str's name."""
    fields = msg.get("fields") or []
    var = None
    fixed = []
    for f in fields:
        if f["type"] == "str":
            if f is not fields[-1]:
                raise SystemExit(f"{msg['name']}: a str field must come last")
            var = f["name"]
        else:
            fixed.append(f)
    return fixed, var


def gen_cpp(spec) -> str:
    L = [
        "// " + BANNER,
        f"// source commit: {git_hash()}",
        "#pragma once",
        "#include <stdint.h>",
        "",
        "namespace kobold {",
        "",
        f"static constexpr uint8_t PROTOCOL_VERSION = {spec['version']};",
        "",
        "// ---- board ids ----",
    ]
    for name, val in spec["boards"].items():
        L.append(f"static constexpr uint8_t BOARD_{name.upper()} = {val};")

    L += ["", "// ---- fault bits ----"]
    for name, val in spec["faults"].items():
        L.append(f"static constexpr uint8_t FAULT_{name.upper()} = 0x{val:02X};")

    L += ["", "// ---- message ids ----", "enum MsgId : uint8_t {"]
    for m in spec["messages"]:
        L.append(f"  MSG_{m['name'].upper()} = 0x{m['id']:02X},")
    L += ["};", "", "// ---- payload layouts (packed, little-endian) ----"]

    for m in spec["messages"]:
        fixed, var = fixed_fields(m)
        if not fixed and not var:
            continue
        if m.get("doc"):
            for line in str(m["doc"]).strip().splitlines():
                L.append(f"// {line.strip()}")
        L.append(f"struct __attribute__((packed)) {camel(m['name'])} {{")
        for f in fixed:
            ctype = TYPES[f["type"]][0]
            comment = f"  // {f['doc']}" if f.get("doc") else ""
            L.append(f"  {ctype} {f['name']};{comment}")
        L.append("};")
        if var:
            L.append(
                f"// NOTE: {m['name']} is followed by a variable-length "
                f"'{var}' field (no NUL terminator)."
            )
        L.append("")

    L += ["}  // namespace kobold", ""]
    return "\n".join(L)


def gen_py(spec) -> str:
    L = [
        '"""' + BANNER,
        "",
        f"source commit: {git_hash()}",
        '"""',
        "",
        "import struct",
        "from typing import NamedTuple",
        "",
        f"PROTOCOL_VERSION = {spec['version']}",
        "",
        "# ---- board ids ----",
    ]
    for name, val in spec["boards"].items():
        L.append(f"BOARD_{name.upper()} = {val}")

    L += ["", "# ---- fault bits ----"]
    for name, val in spec["faults"].items():
        L.append(f"FAULT_{name.upper()} = 0x{val:02X}")

    L += ["", "# ---- message ids ----"]
    for m in spec["messages"]:
        L.append(f"MSG_{m['name'].upper()} = 0x{m['id']:02X}")

    L += [
        "",
        "MSG_NAMES = {",
        *[f"    0x{m['id']:02X}: {m['name']!r}," for m in spec["messages"]],
        "}",
        "",
        "",
    ]

    for m in spec["messages"]:
        fixed, var = fixed_fields(m)
        cls = camel(m["name"])
        if not fixed and not var:
            L += [
                f"class {cls}(NamedTuple):",
                f'    """{m["name"]} — empty payload."""',
                "",
                f"    STRUCT = struct.Struct('<')",
                f"    MSG_ID = MSG_{m['name'].upper()}",
                "",
                "    def pack(self) -> bytes:",
                "        return b''",
                "",
                "    @classmethod",
                "    def unpack(cls, data: bytes) -> '%s':" % cls,
                "        return cls()",
                "",
                "",
            ]
            continue

        fmt = "<" + "".join(TYPES[f["type"]][1] for f in fixed)
        L.append(f"class {cls}(NamedTuple):")
        doc = str(m.get("doc") or m["name"]).strip().replace("\n", " ")
        L.append(f'    """{doc}"""')
        L.append("")
        for f in fixed:
            pytype = "float" if f["type"] == "f32" else "int"
            L.append(f"    {f['name']}: {pytype}")
        if var:
            L.append(f"    {var}: str = ''")
        L += [
            "",
            f"    STRUCT = struct.Struct({fmt!r})",
            f"    MSG_ID = MSG_{m['name'].upper()}",
            "",
            "    def pack(self) -> bytes:",
            "        head = self.STRUCT.pack("
            + ", ".join(f"self.{f['name']}" for f in fixed)
            + ")",
        ]
        if var:
            L.append(f"        return head + self.{var}.encode('utf-8')")
        else:
            L.append("        return head")
        L += [
            "",
            "    @classmethod",
            f"    def unpack(cls, data: bytes) -> '{cls}':",
            "        n = cls.STRUCT.size",
            "        vals = cls.STRUCT.unpack(data[:n])",
        ]
        if var:
            L.append("        return cls(*vals, data[n:].decode('utf-8', 'replace'))")
        else:
            L.append("        return cls(*vals)")
        L += ["", ""]

    L += [
        "# Every message, both directions. The bridge only ever receives the",
        "# dev_to_host subset, but the full map lets tests and the firmware",
        "# simulator decode host_to_dev frames too.",
        "DECODERS = {",
        *[f"    MSG_{m['name'].upper()}: {camel(m['name'])}," for m in spec["messages"]],
        "}",
        "",
        "DEV_TO_HOST = frozenset({",
        *[
            f"    MSG_{m['name'].upper()},"
            for m in spec["messages"]
            if m.get("dir") == "dev_to_host"
        ],
        "})",
        "",
        "HOST_TO_DEV = frozenset({",
        *[
            f"    MSG_{m['name'].upper()},"
            for m in spec["messages"]
            if m.get("dir") == "host_to_dev"
        ],
        "})",
        "",
    ]
    return "\n".join(L)


def main() -> int:
    spec = yaml.safe_load(SPEC.read_text())

    seen = {}
    for m in spec["messages"]:
        if m["id"] in seen:
            sys.exit(f"duplicate message id 0x{m['id']:02X}: {seen[m['id']]} / {m['name']}")
        seen[m["id"]] = m["name"]

    OUT_H.parent.mkdir(parents=True, exist_ok=True)
    OUT_PY.parent.mkdir(parents=True, exist_ok=True)
    OUT_H.write_text(gen_cpp(spec))
    OUT_PY.write_text(gen_py(spec))

    print(f"protocol v{spec['version']}, {len(spec['messages'])} messages")
    print(f"  -> {OUT_H.relative_to(ROOT)}")
    print(f"  -> {OUT_PY.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
