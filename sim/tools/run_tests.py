"""Test runner: uses real pytest/hypothesis when available, shims otherwise.

    python3 tools/run_tests.py              # fast tests only
    python3 tools/run_tests.py --slow       # include the paper-regression suite
    python3 tools/run_tests.py -k truss     # filter by name

With network access, prefer the real libraries:

    uv sync --extra dev && uv run pytest
"""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT / "tests"))


def _use_real_pytest() -> bool:
    try:
        import hypothesis  # noqa: F401
        import pytest  # noqa: F401
        return True
    except ImportError:
        return False


def _run_with_pytest(argv: list[str]) -> int:
    import pytest
    args = [str(ROOT / "tests"), "-q"] + argv
    print("Running with real pytest + hypothesis\n")
    return pytest.main(args)


def _collect(module) -> list[tuple[str, callable]]:
    """Collect test callables from a module: bare functions and test classes."""
    found: list[tuple[str, callable]] = []
    for name in dir(module):
        if name.startswith("Test"):
            cls = getattr(module, name)
            if not isinstance(cls, type):
                continue
            for attr in dir(cls):
                if attr.startswith("test"):
                    found.append((f"{name}.{attr}", (cls, attr)))
        elif name.startswith("test_"):
            obj = getattr(module, name)
            if callable(obj):
                found.append((name, (None, obj)))
    return found


def _expand_parametrize(func):
    """Turn recorded ``@parametrize`` cases into concrete argument dicts."""
    cases = getattr(func, "_shim_parametrize", None)
    if not cases:
        return [({}, "")]

    expanded = [({}, "")]
    for names, values in cases:
        grown = []
        for base, label in expanded:
            for value in values:
                tup = value if isinstance(value, tuple) else (value,)
                merged = dict(base)
                merged.update(dict(zip(names, tup)))
                suffix = f"[{'-'.join(str(v) for v in tup)}]"
                grown.append((merged, label + suffix))
        expanded = grown
    return expanded


def _run_with_shims(include_slow: bool, pattern: str | None) -> int:
    from offline_testkit import install
    install()
    import offline_testkit._pytest as shim_pytest

    print("PyPI unreachable: running with offline_testkit shims")
    print("(random sampling only, no shrinking -- see vendor/offline_testkit)\n")

    modules = [
        "test_transforms_and_dynamics",
        "test_gcode_and_truss",
        "test_compensation",
    ]
    if include_slow:
        modules.append("test_paper_results")

    passed = failed = skipped = 0
    failures: list[tuple[str, str]] = []

    for module_name in modules:
        module = __import__(module_name)
        module_marks = getattr(module, "pytestmark", None)
        module_slow = False
        if module_marks is not None:
            marks = module_marks if isinstance(module_marks, list) else [module_marks]
            module_slow = any(getattr(m, "name", "") == "slow" for m in marks)
        if module_slow and not include_slow:
            continue

        print(f"--- {module_name}")

        # Module-scoped fixtures, computed lazily and shared.
        fixture_cache: dict[str, object] = {}

        def resolve_fixture(name: str):
            if name not in fixture_cache:
                fixture_fn = getattr(module, name, None)
                if fixture_fn is None or not getattr(fixture_fn, "_shim_fixture", False):
                    raise KeyError(f"unknown fixture {name!r}")
                fixture_cache[name] = fixture_fn()
            return fixture_cache[name]

        for label, target in _collect(module):
            cls, attr = target
            if pattern and pattern.lower() not in f"{module_name}.{label}".lower():
                continue

            func = getattr(cls, attr) if cls is not None else attr
            marks = getattr(func, "_shim_marks", set())
            if "slow" in marks and not include_slow:
                skipped += 1
                continue

            import inspect
            signature = inspect.signature(func)
            param_names = [p for p in signature.parameters if p != "self"]

            for extra_args, suffix in _expand_parametrize(func):
                kwargs = dict(extra_args)
                needs_fixture = [p for p in param_names
                                 if p not in kwargs
                                 and getattr(getattr(module, p, None),
                                             "_shim_fixture", False)]
                try:
                    for name in needs_fixture:
                        kwargs[name] = resolve_fixture(name)

                    instance = cls() if cls is not None else None
                    if instance is not None:
                        func(instance, **kwargs)
                    else:
                        func(**kwargs)
                    passed += 1
                except shim_pytest._Skipped:
                    skipped += 1
                except Exception:
                    failed += 1
                    failures.append((f"{module_name}::{label}{suffix}",
                                     traceback.format_exc()))
                    print(f"  FAIL {label}{suffix}")

    print(f"\n{passed} passed, {failed} failed, {skipped} skipped")
    if failures:
        print("\n" + "=" * 72)
        for name, tb in failures:
            print(f"\nFAILED {name}\n{tb}")
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slow", action="store_true",
                        help="include the slow paper-regression tests")
    parser.add_argument("-k", dest="pattern", default=None,
                        help="only run tests whose name contains this")
    args, unknown = parser.parse_known_args()

    if _use_real_pytest():
        argv = list(unknown)
        if not args.slow:
            argv += ["-m", "not slow"]
        if args.pattern:
            argv += ["-k", args.pattern]
        return _run_with_pytest(argv)

    return _run_with_shims(args.slow, args.pattern)


if __name__ == "__main__":
    sys.exit(main())
