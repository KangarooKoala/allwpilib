#!/usr/bin/env python3

import io
import inspect
import pathlib
import re
import subprocess
import sys
import toml

def main():
    base_path: pathlib.Path = pathlib.Path(sys.argv[1])

    out = {}

    for f, name in ((base_path / "units.hpp", "base"), (base_path / "units-usc.hpp", "usc")):
        units: list[tuple[str, str]] = []

        with open(f) as reader:
            in_unit_symbols_namespace: bool = False
            for line in reader:
                line = line.rstrip()

                if line == "namespace unit_symbols {":
                    in_unit_symbols_namespace = True
                elif line == "}  // namespace unit_symbols":
                    in_unit_symbols_namespace = False

                if not in_unit_symbols_namespace:
                    continue

                unit_symbol: str
                prefixes: tuple[str, ...]
                if line.startswith("using "):
                    assert ":" in line
                    assert ";" in line
                    unit_symbol = line[line.rindex(":") + 1 : line.rindex(";")]
                    prefixes = ("",)
                elif (m := re.match(r"ADD_METRIC_UNIT_SYMBOLS\(\w+, (\w+)\)$", line)):
                    unit_symbol = m.group(1)
                    prefixes = ("n", "u", "m", "", "k")
                elif (m := re.match(r"ADD_METRIC_UNIT_SYMBOLS_NO_MICRO\(\w+, (\w+)\)$", line)):
                    unit_symbol = m.group(1)
                    prefixes = ("n", "m", "", "k")
                else:
                    continue

                mp_type: str
                if unit_symbol in ("deg_C", "K"):
                    mp_type = "quantity_point"
                else:
                    mp_type = "quantity"

                units.extend((f"{prefix}{unit_symbol}", mp_type) for prefix in prefixes)

        out_name = f"units_{name}_type_caster.h"

        if units:
            ofp = io.StringIO()

            ofp.write("#pragma once\n")
            ofp.write("\n")
            ofp.write(f"#include <wpi/{f.name}>\n")
            ofp.write("\n")

            ofp.write("\nnamespace pybind11 { namespace detail {\n")

            for unit, mp_type in units:
                ofp.write(
                    inspect.cleandoc(
                        f"""

                    template <> struct handle_type_name<mp::{mp_type}<mp::{unit}>> {{
                    static constexpr auto name = _("{unit}");
                    }};

                """
                    )
                )
                ofp.write("\n\n")

            ofp.write("\n}\n}\n\n")

            ofp.write(f'#include "_units_base_type_caster.h"\n')
            ofp.write("\n")

            ofp.seek(0)

            content = subprocess.check_output(
                ["clang-format"], input=ofp.getvalue(), encoding="utf-8"
            )

            with open(out_name, "w+") as fp:
                fp.write(content)

            print(toml.dumps({
                "tool.semiwrap.export_type_casters.wpimath-casters.headers": {
                    "header": out_name,
                    "types": sorted(
                        [f"mp::{mp_type}<mp::{unit}>" for unit, mp_type in units]
                    ),
                    "default_arg_cast": True
                }
            }))
            # out[out_name] = sorted(
            #     [f"mp::{mp_type}<mp::{unit}>" for unit, mp_type in units]
            # )

    # print(toml.dumps(out))


if __name__ == "__main__":
    main()
