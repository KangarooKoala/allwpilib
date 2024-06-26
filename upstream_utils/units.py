#!/usr/bin/env python3

import os
import shutil

from upstream_utils import (
    walk_cwd_and_copy_if,
    Lib,
)


def copy_upstream_src(wpilib_root):
    upstream_root = os.path.abspath(".")
    wpimath = os.path.join(wpilib_root, "wpimath")

    # Delete old install
    for d in [
        "src/main/native/thirdparty/mp-units/include",
    ]:
        shutil.rmtree(os.path.join(wpimath, d), ignore_errors=True)

    # Copy units code header files into allwpilib
    os.chdir(os.path.join(upstream_root, "src/core"))
    core_include_files = walk_cwd_and_copy_if(
        lambda dp, f: dp.startswith("./include/mp-units"),
        os.path.join(wpimath, "src/main/native/thirdparty/mp-units"),
    )

    # Copy units systems header files into allwpilib
    os.chdir(os.path.join(upstream_root, "src/systems"))
    systems_include_files = walk_cwd_and_copy_if(
        lambda dp, f: dp.startswith("./include/mp-units/systems"),
        os.path.join(wpimath, "src/main/native/thirdparty/mp-units"),
    )


def main():
    name = "units"
    url = "https://github.com/mpusz/mp-units"
    tag = "v2.2.0"

    units = Lib(name, url, tag, copy_upstream_src)
    units.main()


if __name__ == "__main__":
    main()
