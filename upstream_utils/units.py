#!/usr/bin/env python3

import os
import shutil
from pathlib import Path

from upstream_utils import Lib, has_prefix, walk_cwd_and_copy_if


def copy_upstream_src(wpilib_root: Path):
    upstream_root = Path(".").absolute()
    wpimath = wpilib_root / "wpimath"

    # Delete old install
    for d in [
        "src/main/native/thirdparty/mp-units/include",
    ]:
        shutil.rmtree(wpimath / d, ignore_errors=True)

    # Copy units code header files into allwpilib
    os.chdir(upstream_root / "src/core")
    core_include_files = walk_cwd_and_copy_if(
        lambda dp, f: has_prefix(dp, Path("include/mp-units")),
        wpimath / "src/main/native/thirdparty/mp-units",
    )

    # Copy units systems header files into allwpilib
    os.chdir(upstream_root / "src/systems")
    systems_include_files = walk_cwd_and_copy_if(
        lambda dp, f: has_prefix(dp, Path("include/mp-units/systems")),
        wpimath / "src/main/native/thirdparty/mp-units",
    )


def main():
    name = "units"
    url = "https://github.com/mpusz/mp-units"
    tag = "v2.4.0"

    units = Lib(name, url, tag, copy_upstream_src)
    units.main()


if __name__ == "__main__":
    main()
