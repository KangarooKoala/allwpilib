#!/usr/bin/env python3

import os
import sys
import time
import traceback
from argparse import ArgumentParser
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path

UNIT_T_TO_UNIT: dict[str, str] = {
    "scalar": "mp::one",
    "meter": "mp::m",
    "degree": "mp::deg",
    "radian": "mp::rad",
    "second": "mp::s",
    "microsecond": "mp::µs",
    "kilogram": "mp::kg",
    "volt": "mp::V",
    "ampere": "mp::A",
    "ohm": "mp::ohm",
    "square_meter": "mp::m2",
    "meters_per_second": "mp::m / mp::s",
    "meters_per_second_squared": "mp::m / mp::s2",
    "radians_per_second": "mp::rad / mp::s",
    "radians_per_second_squared": "mp::rad / mp::s2",
    "curvature": "mp::rad / mp::m",
    "newton_meter": "mp::N * mp::m",
    "kilogram_square_meter": "mp::kg * mp::m2",
}


BASE_UDL_TO_UNIT: dict[str, str] = {
    "m": "mp::m",
    "s": "mp::s",
    "ms": "mp::ms",
    "rad": "mp::rad",
    "deg": "mp::deg",
    "kg": "mp::kg",
    "V": "mp::V",
    "A": "mp::A",
    "mps": "mp::m / mp::s",
    "mps_sq": "mp::m / mp::s2",
    "rad_per_s": "mp::rad / mp::s",
    "rad_per_s_sq": "mp::rad / mp::s2",
    "deg_per_s": "mp::deg / mp::s",
    "deg_per_s_sq": "mp::deg / mp::s2",
    "rpm": "mp::rev / mp::min",
    "Nm": "mp::N * mp::m",
    "kg_sq_m": "mp::kg * mp::m2",
}


USC_UDL_TO_UNIT: dict[str, str] = {
    "ft": "mp::ft",
    "in": "mp::in",
    "fps": "mp::ft / mp::s",
    "fps_sq": "mp::ft / mp::s2",
}


ALL_UDL_TO_UNIT: dict[str, str] = {
    **BASE_UDL_TO_UNIT,
    **USC_UDL_TO_UNIT,
}


def sub_slice(s: str, start: int, end: int, new: str) -> str:
    """Returns a copy with a slice replaced."""
    return s[:start] + new + s[end:]


def replace(s: str, i: int, old: str, new: str) -> str:
    """Returns a copy with a slice replaced."""
    if s[i : i + len(old)] != old:
        raise ValueError(f"No {old!r} to replace at index {i} of string {s!r}!")
    return sub_slice(s, i, i + len(old), new)


def insert(s: str, i: int, new: str) -> str:
    """Returns a copy with a string inserted."""
    return sub_slice(s, i, i, new)


def delete(s: str, i: int, old: str) -> str:
    """Returns a copy with a slice deleted."""
    return replace(s, i, old, "")


def prev_non_space(lines: list[str], line: int, pos: int) -> tuple[str, int, int]:
    """
    Returns the previous non-space character. Empty string if there is none.
    """
    while line > 0 and pos == 0:
        line -= 1
        pos = len(lines[line])
    if line == 0 and pos == 0:
        return "", -1, -1
    pos -= 1
    while line >= 0:
        s: str = lines[line]
        pos = pos if pos >= 0 else len(s) - 1
        while pos >= 0:
            if not s[pos].isspace():
                return s[pos], line, pos
            pos -= 1
        line -= 1
    return "", -1, -1


def find_match(lines: list[str], line: int, pos: int) -> tuple[int, int]:
    """
    Finds the matching grouping symbol for the symbol at (line, pos).
    """
    start_line: int = line
    opener: str = lines[line][pos]
    lefts: str = "({[<"
    rights: str = ")}]>"
    dir: int
    closer: str
    next_closer: Callable[[str, int], int]
    if opener in lefts:
        dir = 1
        closer = rights[lefts.find(opener)]
        next_closer = lambda s, prev_i: s.find(closer, prev_i + 1)
    elif opener in rights:
        dir = -1
        closer = lefts[rights.find(opener)]
        next_closer = lambda s, prev_i: s.rfind(closer, 0, prev_i)
    else:
        raise ValueError(f"Symbol {opener} is not a grouping symbol")

    def openers_between(s: str, i1: int, i2: int) -> int:
        first: int = min(i1, i2)
        second: int = max(i1, i2)
        return s.count(opener, first + 1, second)

    nests: int = 1
    prev_i: int = pos
    while 0 <= line < len(lines):
        while (i := next_closer(lines[line], prev_i)) >= 0:
            nests -= 1
            nests += openers_between(lines[line], i, prev_i)
            if nests == 0:
                return line, i
            prev_i = i
        nests += openers_between(
            lines[line], prev_i, len(lines[line]) if dir > 0 else -1
        )
        line += dir
        if line < 0:
            break
        prev_i = -1 if dir > 0 else len(lines[line])
    raise ValueError(
        f"No matching {closer} for {opener} at {pos} in {lines[start_line]!r}"
    )


def find_expr_start(lines: list[str], line: int, pos: int) -> tuple[int, int]:
    """Finds the start of the expression that a method is called on. The . is assumed to be at (line, pos + 1)."""
    s: str = lines[line]
    was_ident: bool = False
    while pos > 0:
        # Skip whitespace, skipping lines if necessary
        pre_whitespace_coords: tuple[int, int] = (line, pos + 1)
        while s[pos].isspace():
            pos -= 1
            if pos < 0:
                if line == 0:
                    # Reached beginning of input, so we're done
                    return pre_whitespace_coords
                line -= 1
                s = lines[line]
                pos = len(s) - 1
        # Skip identifiers and numbers
        if not was_ident and (s[pos].isalnum() or s[pos] == "_"):
            while s[pos].isalnum() or s[pos] == "_":
                pos -= 1
            was_ident = True
            continue
        was_ident = False
        # Ensure we don't have post-increment or post-decrement
        if s[pos - 1 : pos + 1] in ("--", "++"):
            raise NotImplementedError("Post- vs pre- increment/decrement not supported")
        # Skip member access and scope resolution
        if s[pos - 1 : pos + 1] in ("->", "::"):
            pos -= 2
            continue
        if s[pos] == ".":
            pos -= 1
            continue
        # Skip over parenthesized atoms, casts, calls, and subscripts
        if s[pos] in ")}]":
            line, pos = find_match(lines, line, pos)
            pos -= 1
            s = lines[line]
            # Skip over function template arguments
            if s[pos : pos + 2] == ">(":
                line, pos = find_match(lines, line, pos)
                pos -= 1
                s = lines[line]
            continue
        # Not in the expression, so we're done
        return pre_whitespace_coords
    if line == 0:
        # Reached beginning of input, so we're done
        return (0, 0)
    raise ValueError("Expression wrapped line outside of grouping symbols")


@dataclass
class NumericLiteral:
    string: str
    start: int
    end: int
    is_int: bool

    @staticmethod
    def from_last(line: str, last: int) -> "NumericLiteral":
        is_int: bool = False
        i: int = last
        # Process first string of digits
        while line[i].isdigit():
            i -= 1
        # Check if this is an integer literal
        is_float: bool = (
            line[i] == "."
            or line[i] in "eE"
            or (line[i - 1] in "eE" and line[i] in "+-")
        )
        # Handle rest of floating point literal; i ends up before first digit
        if is_float:
            # Skip middle section
            while not line[i].isdigit():
                i -= 1
            # Skip digits
            while line[i].isdigit():
                i -= 1
        # Return to first digit
        i += 1
        return NumericLiteral(line, i, last + 1, not is_float)


def is_end_of_unit(lines: list[str], i: int, pos: int) -> bool:
    """Checks if a position is the end of a unit object (either nholthaus or mp)."""
    while lines[i][pos] in ")}":
        # Check for nh type initialization
        new_i, new_pos = find_match(lines, i, pos)
        if lines[new_i][new_pos - 1] == ">":
            new_i, new_pos = find_match(lines, new_i, new_pos - 1)
        new_pos -= 1
        has_preceding_ident: bool = lines[new_i][new_pos].isalpha()
        while new_pos > 0 and not lines[new_i][new_pos].isspace():
            new_pos -= 1
        if lines[new_i][new_pos].isspace():
            new_pos += 1
        if lines[new_i][new_pos:].startswith("units::"):
            return True
        elif lines[i][pos] == "}" or has_preceding_ident:
            # If we're initializing a type or calling a function, we're not in a unit
            return False
        # We're processing some nested expression, try the next level
        pos -= 1
    # Check the last "atom" (Not an actual expression atom, but an expression delineated by whitespace)
    old_pos: int = pos
    while pos > 0 and not lines[i][pos].isspace() and lines[i][pos] not in "({":
        pos -= 1
    while lines[i][pos].isspace() or lines[i][pos] in "({":
        pos += 1
    atom: str = lines[i][pos : old_pos + 1]
    # Check for mp type
    if atom.startswith("mp::"):
        return True
    # Check for numeric UDL
    if atom and atom[0].isdigit() or atom[0] in "-+":
        for nh_udl in ALL_UDL_TO_UNIT:
            udl_suffix: str = f"_{nh_udl}"
            if atom.endswith(udl_suffix):
                return True
    # Check for unit template argument
    if atom and atom[0].isupper() and atom.isalpha():
        print(f"Line {i}: Assuming that {atom} is a unit template argument")
        return True
    return False


def in_unit_composition(
    lines: list[str], start: tuple[int, int], end: tuple[int, int]
) -> bool:
    """Checks if the specified region is inside a unit composition."""
    # Unpack start and end position
    (start_line_i, start_pos) = start
    (end_line_i, end_pos) = end
    # Check if this is rhs of multiplication or division
    prev_c, prev_line_i, prev_pos = prev_non_space(lines, start_line_i, start_pos)
    # Skip past "(", such as in 1_V / (1_rad / 1_s) when the region is 1_rad / 1_s
    while prev_c == "(":
        prev_c, prev_line_i, prev_pos = prev_non_space(lines, prev_line_i, prev_pos)
    # Unit composition requires multiplication or division of unit types
    if prev_c and prev_c in "*/":
        # Go to the end of the lhs
        _, line, pos = prev_non_space(lines, prev_line_i, prev_pos)
        return is_end_of_unit(lines, line, pos)
    return False


def has_top_level_addsub(
    lines: list[str], start: tuple[int, int], end: tuple[int, int]
) -> bool:
    """Checks if the specified region has a top level addition or subtraction operation."""
    (start_line_i, start_pos) = start
    (end_line_i, end_pos) = end
    line_i: int = end_line_i
    pos: int = end_pos - 1
    while line_i >= start_line_i:
        min_pos = start_pos if line_i == start_line_i else 0
        while pos >= min_pos:
            if lines[line_i][pos] in ")}]" or lines[line_i][pos : pos + 2] == ">(":
                line_i, pos = find_match(lines, line_i, pos)
                continue
            if lines[line_i][pos - 1 : pos + 2] in (" + ", " - "):
                return True
            pos -= 1
        line_i -= 1
    return False


def add_scalar_to_unit_conversion(
    lines: list[str], start: tuple[int, int], end: tuple[int, int], unit: str
):
    """Adds a conversion of the specified region from a scalar to the specified unit."""
    (start_line_i, start_pos) = start
    (end_line_i, end_pos) = end
    # Wrap scalar portion in parentheses if necessary
    if has_top_level_addsub(lines, start, end):
        lines[start_line_i] = insert(lines[start_line_i], start_pos, "(")
        if start_line_i == end_line_i:
            end_pos += 1
        lines[end_line_i] = insert(lines[end_line_i], end_pos, ")")
        end_pos += 1
    # Add a decimal if necessary
    if lines[end_line_i][end_pos - 1].isdigit():
        literal: NumericLiteral = NumericLiteral.from_last(
            lines[end_line_i], end_pos - 1
        )
        if literal.is_int and not lines[end_line_i][literal.start - 1].isalpha():
            lines[end_line_i] = insert(lines[end_line_i], end_pos, ".0")
            end_pos += len(".0")
    # Add multiplication to convert to unit
    lines[end_line_i] = insert(lines[end_line_i], end_pos, f" * {unit}")
    unit_end: int = end_pos + len(f" * {unit}")
    # Check if we are in a unit composition
    if (
        start_line_i == end_line_i
        and lines[start_line_i][start_pos:end_pos] == "1.0"
        and in_unit_composition(lines, (start_line_i, start_pos), (end_line_i, end_pos))
    ):
        # Delete "1.0 * "
        lines[end_line_i] = delete(lines[end_line_i], start_pos, "1.0 * ")
        end_pos = start_pos  # No more numeric part
        unit_end -= len("1.0 * ")
    # Wrap unit expression in parentheses if necessary
    prev_c, _, _ = prev_non_space(lines, start_line_i, start_pos)
    if (lines[end_line_i][unit_end] == "." or prev_c == "/") and (
        start_line_i != end_line_i or " " in lines[start_line_i][start_pos:unit_end]
    ):
        lines[start_line_i] = insert(lines[start_line_i], start_pos, "(")
        start_pos += 1
        if start_line_i == end_line_i:
            end_pos += 1
            unit_end += 1
        lines[end_line_i] = insert(lines[end_line_i], unit_end, ")")


def process_unit_type_initialization(
    lines: list[str], i: int, start: int, unit_type: str, unit: str
):
    """Processes a unit type initialization starting at (i, start)."""
    if not lines[i][start:].startswith(unit_type):
        raise ValueError(f"Expected {unit_type} at ({i}, {start})")
    end: int = start + len(unit_type)
    init_pair: str = "{}" if lines[i][end] == "{" else "()"
    # Delete unit type
    lines[i] = delete(lines[i], start, unit_type)
    # Find "}"
    r_line_i, r_pos = find_match(lines, i, start)
    # Delete initial "{"
    lines[i] = delete(lines[i], start, init_pair[0])
    if r_line_i == i:
        r_pos -= 1
    # Delete "}"
    lines[r_line_i] = delete(lines[r_line_i], r_pos, init_pair[1])
    # Add " * unit" or ".in(unit)" at the end
    replacement: str
    if lines[r_line_i][r_pos:].startswith(".value()") or is_end_of_unit(
        lines, r_line_i, r_pos - 1
    ):
        # Unit conversion
        replacement = f".in({unit})"
        lines[r_line_i] = insert(lines[r_line_i], r_pos, replacement)
    else:
        add_scalar_to_unit_conversion(lines, (i, start), (r_line_i, r_pos), unit)


def check_variable_initializer(lines: list[str], i: int, end: int):
    """Warns if the unit type ending at (i, end) is in a variable declaration with an initializer."""
    pos: int = end
    if not lines[i][pos].isspace():
        # Variable declaration must be space between type and variable name
        return
    # Skip spaces
    while pos < len(lines) and lines[i][pos].isspace():
        pos += 1
    if pos >= len(lines):
        # Only check single-line variable declarations
        return
    if not lines[i][pos].isalpha() and lines[i][pos] != "_":
        # Must be identifier
        return
    # Skip identifier
    while lines[i][pos].isalnum() or lines[i][pos] == "_":
        pos += 1
    # TODO Figure out disambiguating function declarations from variable declarations
    # When they're ambiguous (most vexing parse), it's a function, but not all cases with parentheses are ambiguous
    if lines[i][pos] == "{":
        print(f"Line {i}: Quantity variable could be initialized from a scalar")


def process_unit_t(lines: list[str], i: int, *, type_only: bool = False):
    """Processes any unit_t type occurrences in the specified line."""
    for nh_unit, mp_unit in (
        ("unit", ""),
        *UNIT_T_TO_UNIT.items(),
    ):
        unit_t: str = f"units::{nh_unit}_t"
        unit_t_start: int = -1
        while (unit_t_start := lines[i].find(unit_t, unit_t_start + 1)) >= 0:
            unit_type: str = unit_t
            unit_t_end: int = unit_t_start + len(unit_t)
            if nh_unit == "unit":
                # Special case for units::unit_t<...>
                if lines[i][unit_t_end] != "<":
                    raise NotImplementedError
                l_angle_i = unit_t_end
                r_angle_line, r_angle_i = find_match(lines, i, l_angle_i)
                if r_angle_line != i:
                    # Too complicated, leave this for manual fixing
                    print(f"Line {i}: units::unit_t<...> is too complicated!")
                    continue
                mp_unit = lines[i][l_angle_i + 1 : r_angle_i]
                mp_unit = mp_unit.removeprefix("typename ")
                unit_t_end = r_angle_i + 1
                unit_type = lines[i][unit_t_start:unit_t_end]
            if lines[i][unit_t_end] not in "{(":
                # Plain type
                quantity: str = f"mp::quantity<{mp_unit}>"
                lines[i] = replace(lines[i], unit_t_start, unit_type, quantity)
                unit_t_end = unit_t_start + len(quantity)
                # Check if this is a type for a variable that has an initializer
                check_variable_initializer(lines, i, unit_t_end)
            elif not type_only:
                # Type initialization
                process_unit_type_initialization(
                    lines, i, unit_t_start, unit_type, mp_unit
                )


def process_unit(lines: list[str], i: int):
    """Processes any unit type occurences in the specified line."""
    for nh_unit_sing, nh_unit_plural, mp_unit in (
        ("units::meter", "units::meters", "mp::m"),
        ("units::centimeter", "units::centimeters", "mp::cm"),
        ("units::radian", "units::radians", "mp::rad"),
        ("units::degree", "units::degrees", "mp::deg"),
        ("units::second", "units::seconds", "mp::s"),
        ("units::volt", "units::volts", "mp::V"),
        ("units::ampere", "units::amperes", "mp::A"),
        ("units::radian_per_second", "units::radians_per_second", "mp::rad / mp::s"),
    ):
        for nh_unit in (nh_unit_sing, nh_unit_plural):
            unit_start: int = -1
            while (unit_start := lines[i].find(nh_unit, unit_start + 1)) >= 0:
                unit_end: int = unit_start + len(nh_unit)
                if lines[i][unit_end].isalpha() or lines[i][unit_end] == "_":
                    continue
                lines[i] = replace(lines[i], unit_start, nh_unit, mp_unit)


def process_udls(lines: list[str], i: int, *, udl_to_unit: dict[str, str]):
    """Processes any UDLs in the specified line."""
    for nh_udl, mp_unit in udl_to_unit.items():
        udl_suffix: str = f"_{nh_udl}"
        suffix_start: int = 0
        while (suffix_start := lines[i].find(udl_suffix, suffix_start + 1)) >= 0:
            suffix_end: int = suffix_start + len(udl_suffix)
            # Make sure this is actually a numeric UDL
            if (
                not lines[i][suffix_start - 1].isdigit()
                or lines[i][suffix_end].isalpha()
                or lines[i][suffix_end] == "_"
            ):
                continue
            # Delete UDL suffix
            lines[i] = delete(lines[i], suffix_start, udl_suffix)
            suffix_end -= len(udl_suffix)
            # Find start of UDL
            udl_start: int = NumericLiteral.from_last(lines[i], suffix_start - 1).start
            # Include a negative if present
            if udl_start > 0 and lines[i][udl_start - 1] == "-":
                udl_start -= 1
            # Add conversion to unit
            add_scalar_to_unit_conversion(
                lines, (i, udl_start), (i, suffix_end), mp_unit
            )


def process_decltype(lines: list[str], i: int, *, type_only: bool = False):
    """Processes decltype(1.0 * mp::...) occurences."""
    decltype_start: int = -1
    while (decltype_start := lines[i].find("decltype(1.0 * ", decltype_start + 1)) >= 0:
        unit_start: int = decltype_start + len("decltype(1.0 * ")
        end_line, unit_end = find_match(lines, i, decltype_start + len("decltype"))
        decltype_end: int = unit_end + 1
        if lines[i][decltype_end] not in "{(":
            # Plain type
            lines[i] = replace(
                lines[i], decltype_start, "decltype(1.0 * ", "mp::quantity<"
            )
            unit_start += len("mp::quantity<") - len("decltype(1.0 * ")
            if end_line == i:
                unit_end += len("mp::quantity<") - len("decltype(1.0 * ")
                decltype_end += len("mp::quantity<") - len("decltype(1.0 * ")
            lines[end_line] = replace(lines[end_line], unit_end, ")", ">")
            # Check if this is a type for a variable that has an initializer
            check_variable_initializer(lines, i, decltype_end)
        elif not type_only:
            # Type initialization
            # Load data
            if end_line != i:
                print(f"Line {i}: decltype(...) type conversion is too complicated!")
                continue
            decltype: str = lines[i][decltype_start:decltype_end]
            unit: str = lines[i][unit_start:unit_end]
            # Perform actual processing
            process_unit_type_initialization(lines, i, decltype_start, decltype, unit)


def process_value_calls(lines: list[str], i: int):
    """Processes any .value() calls in the specified line."""
    while (index := lines[i].find(".value()")) >= 0:
        # Delete .value()
        lines[i] = delete(lines[i], index, ".value()")
        # Handle parenthesized expression
        prev_ns_c, prev_ns_line, prev_ns_pos = prev_non_space(lines, i, index)
        if prev_ns_c == ")":
            lparen_line_i, lparen_pos = find_match(lines, prev_ns_line, prev_ns_pos)
            lparen_line = lines[lparen_line_i]
            # Make sure this isn't a method call (which can have template arguments)
            if (
                not lparen_line[lparen_pos - 1].isalpha()
                and lparen_line[lparen_pos - 1] != ">"
            ):
                # Add mp::value before lparen
                lines[lparen_line_i] = insert(lparen_line, lparen_pos, "mp::value")
                continue
        start_line_i, start_pos = find_expr_start(lines, i, index - 1)
        # Add rparen
        lines[i] = insert(lines[i], index, ")")
        # Add mp::value with lparen at start
        lines[start_line_i] = insert(lines[start_line_i], start_pos, "mp::value(")


def translate_lines(lines: list[str], *, check_iwyu: bool = True) -> bool:
    """Modifies the provided lines in place to translate from nholthaus units to mp-units"""

    def line_iter() -> Iterable[int]:
        """
        A generator that iterates over the indices of lines.

        Given the most recently yielded index i, any operation (including addition
        or deletion of elements) on lines[:i + 1] is permitted, and the next
        yielded index will always point to the first element that has not be
        processed yet.
        """
        dirty: bool = False
        i: int = 0
        while i < len(lines):
            old_len: int = len(lines)
            old_line = lines[i]
            yield i
            # Adjust for any added or deleted elements
            i += len(lines) - old_len
            # Update dirty
            dirty |= lines[i] != old_line
            # Delete newly blank lines
            if not old_line.isspace() and lines[i].isspace():
                del lines[i]
                i -= 1
            # Advance to next element
            i += 1
        # In a generator, a return value goes into the .value attribute of the
        # raised StopIteration that signals the end of iteration
        return dirty

    # Process includes
    base_include_line: int = -1
    for i in line_iter():
        if lines[i].startswith("#include") and "units/" in lines[i]:
            if base_include_line == -1:
                begin: str = lines[i][lines[i].find("units/") - 1]
                end: str = lines[i].rstrip("\r\n")[-1]
                lines[i] = f"#include {begin}frc/units.h{end}\n"
                base_include_line = i
            else:
                del lines[i]

    # Flags for kinds of changes made
    changed_body: bool = False
    need_base_include: bool = False
    need_usc_include: bool = False

    def body_iter(*, include_kind: str = "base"):
        nonlocal changed_body
        nonlocal need_base_include
        nonlocal need_usc_include
        it = iter(line_iter())
        try:
            while True:
                yield next(it)
        except StopIteration as s:
            changed_body |= s.value
            if include_kind == "base":
                need_base_include |= s.value
            if include_kind == "usc":
                need_usc_include |= s.value

    # We detect function headers as :: followed by an identifier followed by an lparen
    # This technically also matches function calls, but those shouldn't have types, so it's okay to include them
    def function_impl_header_iter():
        last_function_header_line: int = -1
        for i in body_iter(include_kind="none"):
            scope_start: int = -1
            while (scope_start := lines[i].find("::", scope_start + 1)) >= 0:
                scope_end: int = scope_start + len("::")
                pos: int = scope_end
                while lines[i][pos].isalnum() or lines[i][pos] == "_":
                    pos += 1
                if pos != scope_end and lines[i][pos] == "(":
                    line, _ = find_match(lines, i, pos)
                    last_function_header_line = line
            if last_function_header_line == -1:
                continue
            if i == last_function_header_line:
                last_function_header_line = -1
            yield i

    # Process units::math functions
    for i in body_iter():
        if "units::math::max" in lines[i]:
            print(
                f"Line {i}: units::math::max replaced with std::max, <algorithm> include may need to be added"
            )
        if "units::math::min" in lines[i]:
            print(
                f"Line {i}: units::math::min replaced with std::min, <algorithm> include may need to be added"
            )
        lines[i] = (
            lines[i]
            .replace("units::math::max", "std::max")
            .replace("units::math::min", "std::min")
            .replace("units::math", "mp")
        )

    # Process concepts
    for i in body_iter():
        lines[i] = (
            lines[i]
            .replace("units::angle_unit auto", "mp::QuantityOf<mp::angle> auto")
            .replace("units::traits::is_unit_t_v", "mp::Quantity")
        )

    # Process function implementation parameter types separately to avoid counting those changes for IWYU
    for i in function_impl_header_iter():
        process_unit_t(lines, i, type_only=True)

    # Process unit_t types
    for i in body_iter():
        process_unit_t(lines, i)

    # Process unit types
    for i in body_iter():
        process_unit(lines, i)

    # Process base UDLs
    for i in body_iter():
        process_udls(lines, i, udl_to_unit=BASE_UDL_TO_UNIT)

    # Process usc UDLs
    for i in body_iter(include_kind="usc"):
        process_udls(lines, i, udl_to_unit=USC_UDL_TO_UNIT)

    # Process function implementation parameter types separately to avoid counting those changes for IWYU
    for i in function_impl_header_iter():
        process_decltype(lines, i, type_only=True)

    # Process decltype() (from nh UDLs)
    for i in body_iter():
        process_decltype(lines, i)

    # Process .value()
    for i in body_iter():
        process_value_calls(lines, i)

    # Clean up double division
    for i in body_iter():
        lines[i] = lines[i].replace("/ mp::s / mp::s", "/ mp::s2")

    # Warn about remaining instances
    nh_units_count: int = sum(x.count("units::") for x in lines)
    if nh_units_count > 0:
        nh_instances: str = (
            "1 instance" if nh_units_count == 1 else f"{nh_units_count} instances"
        )
        print(f"Note: {nh_instances} of old units remaining that need manual updating")

    # Automatically add frc/units-usc.h include if needed
    if base_include_line != -1 and need_usc_include:
        i: int = base_include_line
        # Copy the original include (so we get the right choice of <> vs "")
        lines.insert(i, lines[i])
        # Change the first of the two lines (since units-usc.h is before units.h)
        assert lines[i].count("units.h") == 1
        lines[i] = lines[i].replace("units.h", "units-usc.h")

    # Do a basic IWYU (Include What You Use) check
    added_include: bool = base_include_line >= 0
    if check_iwyu:
        need_include: bool = need_base_include or need_usc_include
        if added_include and not need_include:
            print("Warning: units headers converted but no changes made to file")
        elif not added_include and need_include:
            print("Warning: Changes made to file but no units includes found")

    return added_include or changed_body


class PrintModifier:
    def __init__(self):
        self._builtin_print = None

    def shimmed_print(self, *args, **kwargs):
        self._builtin_print(*args, **kwargs)

    def stop(self):
        __builtins__.print = self._builtin_print

    def run(self, f: Callable[[], None]):
        self._builtin_print = __builtins__.print
        try:
            __builtins__.print = self.shimmed_print
            return f()
        finally:
            __builtins__.print = self._builtin_print


class ShimmedPrinter(PrintModifier):
    def __init__(self, header: int, indent: str):
        super().__init__()
        self.header = header
        self.indent = indent
        self.printed_header = False

    def shimmed_print(self, *args, **kwargs):
        if kwargs.get("file") is not None:
            self._builtin_print(*args, **kwargs)
            return
        header: str = self.header
        indent: str = self.indent
        if not self.printed_header:
            self._builtin_print(f"{header}")
            self.printed_header = True
        args = (indent,) if not args else (f"{indent}{args[0]!s}", *args[1:])
        self._builtin_print(*args, **kwargs)


class PrintCapturer(PrintModifier):
    def __init__(self):
        super().__init__()
        self.output: list[str] = []

    def shimmed_print(self, *args, **kwargs):
        if kwargs.get("file") is not None:
            self._builtin_print(*args, **kwargs)
            return
        sep: str = kwargs.get("sep", " ")
        end: str = kwargs.get("end", "\n")
        self.output.append(sep.join(args) + end)


def test_translate(
    name: str,
    test_input: tuple[str],
    expected: tuple[str],
    expected_output: tuple[str] = (),
) -> bool:
    lines: list[str] = list(test_input)
    print_capturer = PrintCapturer()
    try:
        dirty = print_capturer.run(lambda: translate_lines(lines, check_iwyu=False))
    except:
        print(f'Error running test "{name}"!')
        raise
    output: list[str] = print_capturer.output
    expected_dirty: bool = test_input != expected
    if (
        lines == list(expected)
        and dirty == expected_dirty
        and output == list(expected_output)
    ):
        return True
    print(f'Test "{name}" failed!')
    if dirty != expected_dirty:
        print(f"Incorrect {dirty = }")
    if lines != list(expected):
        print("Translation result:")
        for line in lines:
            print("> " + line, end="")
        print("Expected:")
        for line in expected:
            print("> " + line, end="")
    if output != list(expected_output):
        print("Translation output:")
        for line in output:
            print("> " + line, end="")
        print("Expected:")
        for line in expected_output:
            print("> " + line, end="")
    return False


def run_tests() -> bool:
    success: bool = True
    success &= test_translate(
        "Quote include",
        (
            '#include "units/area.h"\n',
            '#include "units/length.h"\n',
            '#include "units/volume.h"\n',
        ),
        ('#include "frc/units.h"\n',),
    )
    success &= test_translate(
        "Angle bracket include",
        (
            "#include <units/area.h>\n",
            "#include <units/length.h>\n",
            "#include <units/volume.h>\n",
        ),
        ("#include <frc/units.h>\n",),
    )
    success &= test_translate(
        "Okay includes",
        (
            "#include <utility>\n",
            "#include <wpi/string.h>\n",
            '#include "frc/units.h"\n',
        ),
        (
            "#include <utility>\n",
            "#include <wpi/string.h>\n",
            '#include "frc/units.h"\n',
        ),
    )
    success &= test_translate(
        "add usc include",
        (
            '#include "units/length.h"\n',
            "\n",
            "static constexpr auto x = 1_in;\n",
        ),
        (
            '#include "frc/units-usc.h"\n',
            '#include "frc/units.h"\n',
            "\n",
            "static constexpr auto x = 1.0 * mp::in;\n",
        ),
    )
    success &= test_translate(
        "units::math",
        ("units::math::hypot(x, y)\n",),
        ("mp::hypot(x, y)\n",),
    )
    success &= test_translate(
        "meter", ("units::meter_t x;\n",), ("mp::quantity<mp::m> x;\n",)
    )
    success &= test_translate(
        "meters",
        ("Translation2d(units::meter_t x, units::meter_t y);\n",),
        ("Translation2d(mp::quantity<mp::m> x, mp::quantity<mp::m> y);\n",),
    )
    success &= test_translate(
        "instantiation",
        ("units::meter_t{0}\n",),
        ("0.0 * mp::m\n",),
    )
    success &= test_translate(
        "instantiation exponent",
        ("units::meter_t{1e-9}\n",),
        ("1e-9 * mp::m\n",),
    )
    success &= test_translate(
        "instantiation multi in line",
        ("units::meter_t{x}, units::meter_t{y}\n",),
        ("x * mp::m, y * mp::m\n",),
    )
    success &= test_translate(
        "instantiation multiline",
        (
            "units::meter_t{\n",
            "  double{0}}\n",
        ),
        ("  double{0} * mp::m\n",),
    )
    success &= test_translate(
        "instantiation division",
        ("1 / units::meter_t{x}\n",),
        ("1 / (x * mp::m)\n",),
    )
    success &= test_translate(
        "instantiation unit composition",
        ("1_V / units::unit_t<Distance>{1}\n",),
        ("1.0 * mp::V / Distance\n",),
    )
    success &= test_translate(
        "double instantiation unit composition",
        ("units::volt_t{1} / units::unit_t<Distance>{1}\n",),
        ("1.0 * mp::V / Distance\n",),
    )
    success &= test_translate(
        "instantiation unit composition parenthesized",
        ("4.0_V / (units::unit_t<Distance>{1} / 1_s)\n",),
        ("4.0 * mp::V / (Distance / mp::s)\n",),
        ("Line 0: Assuming that Distance is a unit template argument\n",),
    )
    success &= test_translate(
        "instantiation subtraction",
        ("units::meter_t{x - y}\n",),
        ("(x - y) * mp::m\n",),
    )
    success &= test_translate(
        "instantiation parenthesized addition",
        ("units::meter_t{(x + y) / 2}\n",),
        ("(x + y) / 2.0 * mp::m\n",),
    )
    success &= test_translate(
        "unit conversion",
        ("units::second_t{dt}.value();\n",),
        ("mp::value(dt.in(mp::s));\n",),
    )
    success &= test_translate(
        "unit conversion two",
        ("double{units::second_t{-10_ms} / foo}\n",),
        ("double{(-10.0 * mp::ms).in(mp::s) / foo}\n",),
    )
    success &= test_translate(
        "unit conversion literal",
        ("units::radian_t{90_deg}\n",),
        ("(90.0 * mp::deg).in(mp::rad)\n",),
    )
    success &= test_translate(
        "unit conversion literal value",
        ("units::radian_t{90_deg}.value();\n",),
        ("mp::value((90.0 * mp::deg).in(mp::rad));\n",),
    )
    success &= test_translate(
        "unit_t integer",
        ("units::unit_t<kv_unit>(0)\n",),
        ("0.0 * kv_unit\n",),
    )
    success &= test_translate(
        "unit_t blank line",
        (
            "units::meter_t{\n",
            "  x}\n",
        ),
        ("  x * mp::m\n",),
    )
    success &= test_translate(
        "parenthesized .value()",
        ("(x).value()\n",),
        ("mp::value(x)\n",),
    )
    success &= test_translate(
        "parenthesized .value() multi in line",
        ("(x).value(), (y).value()\n",),
        ("mp::value(x), mp::value(y)\n",),
    )
    success &= test_translate(
        "double value() function",
        ("f(x.value(), y.value())\n",),
        ("f(mp::value(x), mp::value(y))\n",),
    )
    success &= test_translate(
        "return .value()",
        ("return x.value();\n",),
        ("return mp::value(x);\n",),
    )
    success &= test_translate(
        "multiline .value()",
        (
            "(x * (y /\n",
            " z))\n",
            "    .value()\n",
        ),
        (
            "mp::value(x * (y /\n",
            " z))\n",
        ),
    )
    success &= test_translate(
        "UDL m",
        ("0_m\n",),
        ("0.0 * mp::m\n",),
    )
    success &= test_translate(
        "UDL decimal",
        ("0.5_m\n",),
        ("0.5 * mp::m\n",),
    )
    success &= test_translate(
        "UDL mps",
        ("0_mps\n",),
        ("0.0 * mp::m / mp::s\n",),
    )
    success &= test_translate(
        "UDL exponent",
        ("1e-9_m\n",),
        ("1e-9 * mp::m\n",),
    )
    success &= test_translate(
        "UDL division",
        ("1 / 2_rad\n",),
        ("1 / (2.0 * mp::rad)\n",),
    )
    success &= test_translate(
        "UDL simple unit composition",
        ("1_V / 1_s\n",),
        ("1.0 * mp::V / mp::s\n",),
    )
    success &= test_translate(
        "UDL unit composition",
        ("1_V / 1_mps\n",),
        ("1.0 * mp::V / (mp::m / mp::s)\n",),
    )
    success &= test_translate(
        "unit plural",
        ("units::meters\n",),
        ("mp::m\n",),
    )
    success &= test_translate(
        "unit singular",
        ("units::meter\n",),
        ("mp::m\n",),
    )
    success &= test_translate(
        "decltype",
        ("decltype(1_V / 1_mps)\n",),
        ("mp::quantity<mp::V / (mp::m / mp::s)>\n",),
    )
    success &= test_translate(
        "decltype unit instantiation",
        ("decltype(1_V / 1_mps){x}\n",),
        ("x * mp::V / (mp::m / mp::s)\n",),
    )
    success &= test_translate(
        "decltype unit conversion",
        ("decltype(1_V / 1_mps){x}.value()\n",),
        ("mp::value(x.in(mp::V / (mp::m / mp::s)))\n",),
    )
    success &= test_translate(
        "decltype multi in line",
        ("decltype(1_V / 1_mps) x, decltype(1_V / 1_mps_sq) y\n",),
        (
            "mp::quantity<mp::V / (mp::m / mp::s)> x, mp::quantity<mp::V / (mp::m / mp::s2)> y\n",
        ),
    )
    if success:
        print("All tests succeeded!")
    return success


def translate_file(outer_printer: ShimmedPrinter, path: Path) -> bool:
    lines: list[str]
    try:
        with open(path) as reader:
            lines = reader.readlines()
    except:
        outer_printer.stop()
        print(f"Error loading {path}!")
        traceback.print_exc()
        return False
    parent_dir: str = path.parent.name
    fn: str = path.name
    is_serde_impl: bool = (
        parent_dir == "proto"
        and (fn.endswith("Proto.cpp") or fn.endswith("ProtoTest.cpp"))
    ) or (
        parent_dir == "struct"
        and (fn.endswith("Struct.cpp") or fn.endswith("StructTest.cpp"))
    )
    try:
        dirty: bool = ShimmedPrinter(f"{path!s}:", "  ").run(
            lambda: translate_lines(lines, check_iwyu=not is_serde_impl)
        )
    except Exception as e:
        outer_printer.stop()
        print(f"Error processing {path}!")
        traceback.print_exc()
        return False
    if dirty:
        with open(path, "w") as writer:
            writer.writelines(lines)
    return dirty


def run_conversions(paths: list[Path]):
    start_time: float = time.monotonic()
    printer: ShimmedPrinter = ShimmedPrinter("  Skipped paths:", "    ")
    files: list[Path] = []
    for path in paths:
        for dp, dn, fn in os.walk(path):
            dp = Path(dp)
            fn.sort()
            if fn and "native" not in dp.parts:
                files_str: str = "1 file" if len(fn) == 1 else f"{len(fn)} files"
                printer.run(lambda: print(f"{files_str} in non-native directory {dp}"))
            else:
                for f in fn:
                    if f.startswith(".") or f in (
                        "units.h",
                        "units-usc.h",
                        "UnitsTest.cpp",
                    ):
                        printer.run(lambda: print(f"file {dp / f}"))
                        continue
                    if not any(
                        f.endswith(ext) for ext in (".h", ".cpp", ".inc", ".hpp", ".c")
                    ):
                        printer.run(lambda: print(f"non-C++ file {dp / f}"))
                        continue
                    files.append(dp / f)
            dn.sort()
            for bad_dir in ("java", "generated", "units", "thirdparty"):
                if bad_dir in dn:
                    printer.run(lambda: print(f"directory {dp / bad_dir}"))
                    dn.remove(bad_dir)
    printer = ShimmedPrinter("  Output:", "    ")
    changed_files: int = 0
    for fp in files:
        changed_file: bool = printer.run(lambda: translate_file(printer, fp))
        if changed_file:
            changed_files += 1
    end_time: float = time.monotonic()
    duration: float = end_time - start_time
    print(
        f"  Finished processing {len(files)} files ({changed_files} files changed) in {duration:.2f} seconds"
    )


def main():
    parser: ArgumentParser = ArgumentParser()
    parser.add_argument("files", nargs="*", type=Path)
    parser.add_argument("--run-tests", action="store_true")
    args = parser.parse_args()
    if args.run_tests:
        print("Running tests")
        success: bool = run_tests()
        if not success:
            sys.exit(1)
    else:
        print("Running conversions")
        run_conversions(args.files)


if __name__ == "__main__":
    main()
