#!/usr/bin/env python3
"""
generate_csv.py — Generate GEN_CSV C++ boilerplate from a YAML schema.

Usage:
    python3 generate_csv.py schema.yaml          # prints to stdout
    python3 generate_csv.py schema.yaml -o out.cpp
"""

import sys
import argparse
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# YAML parser (no external dependencies)
# ---------------------------------------------------------------------------

@dataclass
class EnumDef:
    name: str
    values: list[str]

@dataclass
class StructDef:
    name: str
    fields: list[str]


def parse_yaml(text: str) -> list[EnumDef | StructDef]:
    """
    Minimal parser for the subset of YAML used in the schema:
        enum <Name>:
            VALUE1
            VALUE2
        struct <Name>:
            field1
            field2
    """
    results = []
    current = None

    for raw_line in text.splitlines():
        line = raw_line.rstrip()

        # Skip empty lines and comments
        if not line.strip() or line.strip().startswith("#"):
            continue

        indent = len(line) - len(line.lstrip())

        if indent == 0:
            # Top-level declaration
            stripped = line.strip().rstrip(":")
            if stripped.startswith("enum "):
                name = stripped[len("enum "):].strip()
                current = EnumDef(name=name, values=[])
                results.append(current)
            elif stripped.startswith("struct "):
                name = stripped[len("struct "):].strip()
                current = StructDef(name=name, fields=[])
                results.append(current)
            else:
                raise ValueError(f"Unexpected top-level line: {repr(line)}")
        else:
            # Indented member
            if current is None:
                raise ValueError(f"Indented line with no parent: {repr(line)}")
            member = line.strip()
            if isinstance(current, EnumDef):
                current.values.append(member)
            elif isinstance(current, StructDef):
                if "[%" in member:
                    base_field, size = member[:-1].split("[%")
                    size = int(size)
                    for idx in range(size):
                        current.fields.append(f"{base_field}[{idx}]")
                else:
                    current.fields.append(member)

    return results


# ---------------------------------------------------------------------------
# Code generators
# ---------------------------------------------------------------------------

def gen_enum(e: EnumDef) -> str:
    lines = []
    lines.append(f"GEN_CSV({e.name}) {{")
    lines.append(f"    if (!is_first) stream << \",\";")
    lines.append(f"    if (header) stream << prefix;")
    lines.append(f"    else {{")
    lines.append(f"        switch (obj) {{")
    for value in e.values:
        lines.append(f"            case {e.name}::{value}:")
        lines.append(f"                stream << \"{value}\";")
        lines.append(f"                break;")
    lines.append(f"            default:")
    lines.append(f"                stream << \"<unknown>\";")
    lines.append(f"                break;")
    lines.append(f"        }}")
    lines.append(f"    }}")
    lines.append(f"}}")
    return "\n".join(lines)


def gen_struct(s: StructDef) -> str:
    lines = []
    lines.append(f"GEN_CSV({s.name}) {{")
    for field in s.fields:
        if " " in field:
            typ, fld = field.split(" ")
            lines.append(f"    GENERATE_FIXED_FIELD({typ}, {fld});")
        else:
            lines.append(f"    GENERATE_FIELD({field});")
    lines.append(f"}}")
    return "\n".join(lines)


def generate(defs: list[EnumDef | StructDef]) -> str:
    blocks = [
        "#include \"csv_utils.hpp\""
    ]
    for d in defs:
        if isinstance(d, EnumDef):
            blocks.append(gen_enum(d))
        elif isinstance(d, StructDef):
            blocks.append(gen_struct(d))
    return "\n\n".join(blocks) + "\n"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Generate GEN_CSV C++ boilerplate from YAML schema.")
    parser.add_argument("input", help="Input YAML file")
    parser.add_argument("-o", "--output", help="Output file (default: stdout)")
    args = parser.parse_args()

    with open(args.input, "r") as f:
        text = f.read()

    defs = parse_yaml(text)
    output = generate(defs)

    if args.output:
        with open(args.output, "w") as f:
            f.write(output)
        print(f"Written to {args.output}", file=sys.stderr)
    else:
        print(output, end="")


if __name__ == "__main__":
    main()