#!/usr/bin/env python3

import re
import os
import os.path
import sys
import time

from pathlib import Path


IGNORE_NAMES: frozenset[str] = frozenset(
    (
        # wpimath
        "k180deg",
        "kAAngular",
        "kALinear",
        "kVAngular",
        "kVLinear",
        # wpinet
        "kDNSServiceErr_NoError",
        "kDNSServiceFlagsAdd",
        "kDNSServiceProtocol_IPv4",
    )
)


def convert_name(name: str) -> tuple[str, bool]:
    new_name: str = re.sub(r"([a-z])([A-Z])", r"\1_\2", name[1:]).upper()
    is_not_simple: bool = bool(re.search(r"[A-Z][A-Z]", name) or re.search("[0-9]", name))
    return (new_name, is_not_simple)


def make_substitution_table() -> dict[str, str]:
    def normal_substitutions(*old_constant_names: tuple[str, ...]) -> dict[str, str]:
        ret: dict[str, str] = {}
        for name in old_constant_names:
            new_name, is_not_simple = convert_name(name)
            assert not is_not_simple
            ret[name] = new_name
        return ret

    base_dict: dict[str, str] = {
        **normal_substitutions(
            # wpiutil color
            'kAliceBlue',
            'kAntiqueWhite',
            'kAqua',
            'kAquamarine',
            'kAzure',
            'kBeige',
            'kBisque',
            'kBlack',
            'kBlanchedAlmond',
            'kBlue',
            'kBlueViolet',
            'kBrown',
            'kBurlywood',
            'kCadetBlue',
            'kChartreuse',
            'kChocolate',
            'kCoral',
            'kCornflowerBlue',
            'kCornsilk',
            'kCrimson',
            'kCyan',
            'kDarkBlue',
            'kDarkCyan',
            'kDarkGoldenrod',
            'kDarkGray',
            'kDarkGreen',
            'kDarkKhaki',
            'kDarkMagenta',
            'kDarkOliveGreen',
            'kDarkOrange',
            'kDarkOrchid',
            'kDarkRed',
            'kDarkSalmon',
            'kDarkSeaGreen',
            'kDarkSlateBlue',
            'kDarkSlateGray',
            'kDarkTurquoise',
            'kDarkViolet',
            'kDeepPink',
            'kDeepSkyBlue',
            'kDenim',
            'kDimGray',
            'kDodgerBlue',
            'kFirebrick',
            'kFirstBlue',
            'kFirstRed',
            'kFloralWhite',
            'kForestGreen',
            'kFuchsia',
            'kGainsboro',
            'kGhostWhite',
            'kGold',
            'kGoldenrod',
            'kGray',
            'kGreen',
            'kGreenYellow',
            'kHoneydew',
            'kHotPink',
            'kIndianRed',
            'kIndigo',
            'kIvory',
            'kKhaki',
            'kLavender',
            'kLavenderBlush',
            'kLawnGreen',
            'kLemonChiffon',
            'kLightBlue',
            'kLightCoral',
            'kLightCyan',
            'kLightGoldenrodYellow',
            'kLightGray',
            'kLightGreen',
            'kLightPink',
            'kLightSalmon',
            'kLightSeaGreen',
            'kLightSkyBlue',
            'kLightSlateGray',
            'kLightSteelBlue',
            'kLightYellow',
            'kLime',
            'kLimeGreen',
            'kLinen',
            'kMagenta',
            'kMaroon',
            'kMediumAquamarine',
            'kMediumBlue',
            'kMediumOrchid',
            'kMediumPurple',
            'kMediumSeaGreen',
            'kMediumSlateBlue',
            'kMediumSpringGreen',
            'kMediumTurquoise',
            'kMediumVioletRed',
            'kMidnightBlue',
            'kMintcream',
            'kMistyRose',
            'kMoccasin',
            'kNavajoWhite',
            'kNavy',
            'kOldLace',
            'kOlive',
            'kOliveDrab',
            'kOrange',
            'kOrangeRed',
            'kOrchid',
            'kPaleGoldenrod',
            'kPaleGreen',
            'kPaleTurquoise',
            'kPaleVioletRed',
            'kPapayaWhip',
            'kPeachPuff',
            'kPeru',
            'kPink',
            'kPlum',
            'kPowderBlue',
            'kPurple',
            'kRed',
            'kRosyBrown',
            'kRoyalBlue',
            'kSaddleBrown',
            'kSalmon',
            'kSandyBrown',
            'kSeaGreen',
            'kSeashell',
            'kSienna',
            'kSilver',
            'kSkyBlue',
            'kSlateBlue',
            'kSlateGray',
            'kSnow',
            'kSpringGreen',
            'kSteelBlue',
            'kTan',
            'kTeal',
            'kThistle',
            'kTomato',
            'kTurquoise',
            'kViolet',
            'kWheat',
            'kWhite',
            'kWhiteSmoke',
            'kYellow',
            'kYellowGreen',
        )
    }

    sorted_dict: dict[str, str] = {}
    for k, v in reversed(sorted(base_dict.items())):
        assert re.escape(k) == k
        assert re.escape(v) == v
        assert k not in IGNORE_NAMES
        sorted_dict[k] = v
    return sorted_dict


SUBSTITUTIONS: dict[str, str] = make_substitution_table()


old_constant_names: set[str] = set()


def perform_substitutions(content: str) -> str:
    for constant_name in frozenset(re.findall(r"\bk[A-Z0-9]\w+\b", content)):
        if constant_name in IGNORE_NAMES:
            continue
        if constant_name in SUBSTITUTIONS:
            new_constant_name: str = SUBSTITUTIONS[constant_name]
            content = re.sub(rf"\b{constant_name}\b", new_constant_name, content)
            continue
        old_constant_names.add(constant_name)

    return content


def files_in_directory(dirpath: Path, filenames: list[str], *, verbose: bool = True):
    if not filenames:
        return
    # Detect directory type
    valid_exts: tuple[str, ...] = ()
    kind: str
    for i, part in enumerate(reversed(dirpath.parts)):
        i = len(dirpath.parts) - 1 - i
        if part == "java":
            valid_exts = (".java",)
            kind = "Java"
            break
        elif part == "semiwrap":
            valid_exts = (".yml",)
            kind = "Semiwrap"
            break
        elif part == "python":
            valid_exts = (".py")
            kind = "Python"
            break
        elif part == "objcpp":
            valid_exts = (".mm", ".hpp")
            kind = "Objective-C++"
            break
        elif part in ("cpp", "native") or part == "src" and "python" in dirpath.parts[:i]:
            # cpp is included to handle .../python/cpp/... paths
            # We also handle .../python/.../src/... paths
            valid_exts = (".c", ".cpp", ".cpp.inl", ".h", ".hpp", ".inc")
            kind = "C++"
            break
        elif part == "generate" and dirpath.parts[i - 1] == "src":
            valid_exts = (".json",)
            kind = "Generate"
            break
    if not valid_exts:
        quiet: bool = False
        if "src" not in dirpath.parts:
            quiet = True
        if len(dirpath.parts) == 4 and dirpath.parts[1] == "src" and dirpath.parts[-1] == "proto":
            # Something like wpiutil/src/main/proto
            quiet = True
        if verbose or not quiet:
            skipped_files_str: str = "1 file" if len(filenames) == 1 else f"{len(filenames)} files"
            print(f"Could not detect directory type for {dirpath}! Skipping {skipped_files_str}")
        return
    for f in filenames:
        if f.startswith("."):
            continue
        ending_check_f: str = f.removesuffix(".jinja") if "generate" in dirpath.parts else f
        if not any(ending_check_f.endswith(ext) for ext in valid_exts):
            quiet: bool = False
            quiet_exts: tuple[str, ...] = (".jpg", ".md", ".png")
            if any(f.endswith(quiet_ext) for quiet_ext in quiet_exts):
                quiet = True
            if kind == "Python" and (f.endswith(".toml") or f == "py.typed"):
                quiet = True
            if verbose or not quiet:
                print(f"Skipping non-{kind} file {dirpath / f}")
            continue
        if f in (
            "StringExtras.hpp",
            "StringExtras.cpp",
            "MemoryBuffer.cpp",
            "MemoryBuffer.hpp",
            "SmallVectorMemoryBuffer.hpp",
        ):
            print(f"Skipping LLVM file {dirpath / f}")
            continue
        yield dirpath / f


def files(*paths: tuple[str], verbose: bool = True):
    for path in paths:
        if os.path.isfile(path):
            # Explicitly specified file
            yield Path(path)
            continue
        for dp, dn, fn in os.walk(path):
            dp = Path(dp)
            # Report files
            yield from files_in_directory(dp, fn, verbose=verbose)
            # Control which directories we recurse into
            dn.sort()
            for bad_dir in ("resources", "thirdparty"):
                if bad_dir in dn:
                    if verbose:
                        print(f"Skipping bad directory {dp / bad_dir}")
                    dn.remove(bad_dir)


def main():
    VERBOSE: bool = False
    start: float = time.monotonic()

    total_file_count: int = 0
    changed_file_count: int = 0
    for file in files(*sys.argv[1:], verbose=VERBOSE):
        content: str
        try:
            with open(file, "r") as f:
                content = f.read()
        except:
            print(f"Error processing {file}!")
            raise

        old_content: str = content
        content = perform_substitutions(old_content)

        if content != old_content:
            changed_file_count += 1

        with open(file, "w") as f:
            f.write(content)

        total_file_count += 1

    end: float = time.monotonic()

    # Produce suggestions

    old_constant_names.difference_update(IGNORE_NAMES)

    complex_names: list[tuple[str, str]] = []
    simple_names: list[tuple[str, str]] = []
    for name in old_constant_names:
        new_name, is_not_simple = convert_name(name)
        (simple_names, complex_names)[is_not_simple].append((name, new_name))
    complex_names.sort()
    simple_names.sort()
    if complex_names:
        print("Complex names:")
        for old_name, new_name in complex_names:
            print(f"        {old_name!r}: {new_name!r},")
    if simple_names:
        print("Simple names:")
        for old_name, new_name in simple_names:
            print(f"            {old_name!r},")

    print(f"Visited {total_file_count} files ({changed_file_count} changed) in {end - start:.2f} s")
    # print(f"{chr(0x1b)}[43mTODO{chr(0x1b)}[m ...")


if __name__ == "__main__":
    main()
