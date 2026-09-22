"""A very small subset of the pytest API. See ``__init__.py`` for why."""

from __future__ import annotations

import math
from typing import Any


class _Approx:
    """Stand-in for ``pytest.approx``."""

    # numpy would otherwise handle ``ndarray == approx(...)`` itself and return
    # an elementwise array, which is ambiguous in an ``assert``. These two hooks
    # make numpy defer to __eq__ below so a single bool comes back.
    __array_priority__ = 1000
    __array_ufunc__ = None

    def __init__(self, expected: Any, rel: float | None = None,
                 abs: float | None = None) -> None:
        self.expected = expected
        self.rel = rel
        self.abs = abs

    def _tolerance(self, expected: float) -> float:
        if self.abs is not None and self.rel is not None:
            return max(self.abs, self.rel * builtins_abs(expected))
        if self.abs is not None:
            return self.abs
        if self.rel is not None:
            return self.rel * builtins_abs(expected)
        return max(1e-12, 1e-6 * builtins_abs(expected))

    def _compare_scalar(self, actual: float, expected: float) -> bool:
        if math.isnan(actual) or math.isnan(expected):
            return False
        if math.isinf(expected) or math.isinf(actual):
            return actual == expected
        return builtins_abs(actual - expected) <= self._tolerance(expected)

    def __eq__(self, actual: Any) -> bool:
        expected = self.expected
        if isinstance(expected, dict):
            if not isinstance(actual, dict) or set(actual) != set(expected):
                return False
            return all(self._compare_scalar(float(actual[k]), float(expected[k]))
                       for k in expected)
        if hasattr(expected, "__len__") and not isinstance(expected, (str, bytes)):
            try:
                if len(actual) != len(expected):
                    return False
            except TypeError:
                return False
            # float() on each element keeps numpy arrays working elementwise
            # and returns a plain bool, not a numpy array.
            return all(self._compare_scalar(float(a), float(e))
                       for a, e in zip(actual, expected))
        return self._compare_scalar(float(actual), float(expected))

    def __ne__(self, actual: Any) -> bool:
        return not self.__eq__(actual)

    def __req__(self, actual: Any) -> bool:
        return self.__eq__(actual)

    def __repr__(self) -> str:
        return f"approx({self.expected!r}, rel={self.rel}, abs={self.abs})"


builtins_abs = abs


def approx(expected: Any, rel: float | None = None,
           abs: float | None = None) -> _Approx:
    return _Approx(expected, rel=rel, abs=abs)


class _Raises:
    """Stand-in for ``pytest.raises``, usable only as a context manager."""

    def __init__(self, expected_exception, match: str | None = None) -> None:
        self.expected_exception = expected_exception
        self.match = match
        self.value: BaseException | None = None

    def __enter__(self) -> "_Raises":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        if exc_type is None:
            raise AssertionError(
                f"DID NOT RAISE {self.expected_exception!r}")
        if not issubclass(exc_type, self.expected_exception):
            return False
        self.value = exc
        if self.match is not None:
            import re
            if not re.search(self.match, str(exc)):
                raise AssertionError(
                    f"pattern {self.match!r} not found in {str(exc)!r}")
        return True


def raises(expected_exception, match: str | None = None) -> _Raises:
    return _Raises(expected_exception, match=match)


def fail(reason: str = "") -> None:
    raise AssertionError(reason)


def skip(reason: str = "") -> None:
    raise _Skipped(reason)


class _Skipped(Exception):
    pass


class _MarkDecorator:
    """Records marks and parametrisation on test functions."""

    def __init__(self, name: str) -> None:
        self.name = name

    def __call__(self, *args, **kwargs):
        # Bare use: @pytest.mark.slow applied directly to a function.
        if len(args) == 1 and callable(args[0]) and not kwargs:
            func = args[0]
            marks = getattr(func, "_shim_marks", set())
            marks.add(self.name)
            func._shim_marks = marks
            return func

        if self.name == "parametrize":
            argnames, argvalues = args[0], args[1]
            names = ([n.strip() for n in argnames.split(",")]
                     if isinstance(argnames, str) else list(argnames))

            def decorator(func):
                cases = getattr(func, "_shim_parametrize", [])
                cases.append((names, list(argvalues)))
                func._shim_parametrize = cases
                return func
            return decorator

        def decorator(func):
            marks = getattr(func, "_shim_marks", set())
            marks.add(self.name)
            func._shim_marks = marks
            return func
        return decorator


class _MarkNamespace:
    def __getattr__(self, name: str) -> _MarkDecorator:
        return _MarkDecorator(name)


mark = _MarkNamespace()


def fixture(func=None, *, scope: str = "function", **kwargs):
    """Mark a function as a fixture. Only the name and scope are honoured."""
    def wrap(f):
        f._shim_fixture = True
        f._shim_fixture_scope = scope
        return f
    return wrap(func) if func is not None else wrap
