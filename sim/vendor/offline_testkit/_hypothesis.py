"""A very small subset of the hypothesis API. See ``__init__.py`` for why.

This performs plain random sampling. It has none of hypothesis's shrinking,
example database, targeted generation or coverage guidance, so it will find
strictly fewer bugs. It exists only so the property tests can run offline.
"""

from __future__ import annotations

import functools
import math
import random
from typing import Any, Callable, Iterable


class _UnsatisfiedAssumption(Exception):
    """Raised by ``assume`` to discard a generated example."""


def assume(condition: bool) -> None:
    if not condition:
        raise _UnsatisfiedAssumption()


class SearchStrategy:
    """Base strategy: knows how to draw one example from a random source."""

    def draw(self, rnd: random.Random) -> Any:
        raise NotImplementedError

    def map(self, fn: Callable[[Any], Any]) -> "SearchStrategy":
        return _MappedStrategy(self, fn)

    def filter(self, predicate: Callable[[Any], bool]) -> "SearchStrategy":
        return _FilteredStrategy(self, predicate)


class _MappedStrategy(SearchStrategy):
    def __init__(self, inner: SearchStrategy, fn: Callable[[Any], Any]) -> None:
        self.inner, self.fn = inner, fn

    def draw(self, rnd: random.Random) -> Any:
        return self.fn(self.inner.draw(rnd))


class _FilteredStrategy(SearchStrategy):
    def __init__(self, inner: SearchStrategy,
                 predicate: Callable[[Any], bool]) -> None:
        self.inner, self.predicate = inner, predicate

    def draw(self, rnd: random.Random) -> Any:
        for _ in range(100):
            value = self.inner.draw(rnd)
            if self.predicate(value):
                return value
        raise _UnsatisfiedAssumption()


class _Floats(SearchStrategy):
    def __init__(self, min_value: float | None = None,
                 max_value: float | None = None,
                 allow_nan: bool = True,
                 allow_infinity: bool = True) -> None:
        self.min_value = -1e6 if min_value is None else float(min_value)
        self.max_value = 1e6 if max_value is None else float(max_value)
        self.allow_nan = allow_nan and min_value is None and max_value is None
        self.allow_infinity = (allow_infinity and min_value is None
                               and max_value is None)

    def draw(self, rnd: random.Random) -> float:
        # Bias toward boundaries and zero, where bugs cluster.
        roll = rnd.random()
        if roll < 0.06:
            return self.min_value
        if roll < 0.12:
            return self.max_value
        if roll < 0.18 and self.min_value <= 0.0 <= self.max_value:
            return 0.0
        if roll < 0.20 and self.allow_nan:
            return math.nan
        if roll < 0.22 and self.allow_infinity:
            return rnd.choice([math.inf, -math.inf])
        return rnd.uniform(self.min_value, self.max_value)


class _Integers(SearchStrategy):
    def __init__(self, min_value: int = -(2 ** 31),
                 max_value: int = 2 ** 31) -> None:
        self.min_value, self.max_value = int(min_value), int(max_value)

    def draw(self, rnd: random.Random) -> int:
        roll = rnd.random()
        if roll < 0.1:
            return self.min_value
        if roll < 0.2:
            return self.max_value
        return rnd.randint(self.min_value, self.max_value)


class _Tuples(SearchStrategy):
    def __init__(self, *strategies: SearchStrategy) -> None:
        self.strategies = strategies

    def draw(self, rnd: random.Random) -> tuple:
        return tuple(s.draw(rnd) for s in self.strategies)


class _Lists(SearchStrategy):
    def __init__(self, elements: SearchStrategy, min_size: int = 0,
                 max_size: int = 16) -> None:
        self.elements = elements
        self.min_size, self.max_size = int(min_size), int(max_size)

    def draw(self, rnd: random.Random) -> list:
        size = rnd.randint(self.min_size, self.max_size)
        return [self.elements.draw(rnd) for _ in range(size)]


class _SampledFrom(SearchStrategy):
    def __init__(self, values: Iterable[Any]) -> None:
        self.values = list(values)

    def draw(self, rnd: random.Random) -> Any:
        return rnd.choice(self.values)


class _Booleans(SearchStrategy):
    def draw(self, rnd: random.Random) -> bool:
        return rnd.random() < 0.5


class _JustStrategy(SearchStrategy):
    def __init__(self, value: Any) -> None:
        self.value = value

    def draw(self, rnd: random.Random) -> Any:
        return self.value


class _Strategies:
    """Implementation of the ``hypothesis.strategies`` namespace."""

    @staticmethod
    def floats(min_value=None, max_value=None, allow_nan=True,
               allow_infinity=True, **kwargs) -> SearchStrategy:
        return _Floats(min_value, max_value, allow_nan, allow_infinity)

    @staticmethod
    def integers(min_value=-(2 ** 31), max_value=2 ** 31, **kwargs) -> SearchStrategy:
        return _Integers(min_value, max_value)

    @staticmethod
    def tuples(*strategies: SearchStrategy) -> SearchStrategy:
        return _Tuples(*strategies)

    @staticmethod
    def lists(elements: SearchStrategy, min_size=0, max_size=16,
              **kwargs) -> SearchStrategy:
        return _Lists(elements, min_size, max_size)

    @staticmethod
    def sampled_from(values: Iterable[Any]) -> SearchStrategy:
        return _SampledFrom(values)

    @staticmethod
    def booleans() -> SearchStrategy:
        return _Booleans()

    @staticmethod
    def just(value: Any) -> SearchStrategy:
        return _JustStrategy(value)


def _build_strategies_module():
    """Expose :class:`_Strategies` as a real module object.

    ``from hypothesis.strategies import floats`` requires a genuine module in
    ``sys.modules``, so the static methods are copied onto one.
    """
    import types

    module = types.ModuleType("hypothesis.strategies")
    for name in dir(_Strategies):
        if not name.startswith("_"):
            setattr(module, name, getattr(_Strategies, name))
    module.SearchStrategy = SearchStrategy
    return module


strategies = _build_strategies_module()

DEFAULT_MAX_EXAMPLES = 60


def settings(max_examples: int = DEFAULT_MAX_EXAMPLES, deadline=None, **kwargs):
    """Record a per-test example budget."""
    def decorator(func):
        func._shim_max_examples = max_examples
        return func
    return decorator


def given(*arg_strategies: SearchStrategy, **kwarg_strategies: SearchStrategy):
    """Run the wrapped test against randomly drawn examples."""
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            budget = getattr(func, "_shim_max_examples", DEFAULT_MAX_EXAMPLES)
            budget = min(budget, 120)  # keep the offline suite quick
            rnd = random.Random(0xC0FFEE ^ hash(func.__qualname__) & 0xFFFFFFFF)

            drawn = 0
            attempts = 0
            while drawn < budget and attempts < budget * 12:
                attempts += 1
                try:
                    positional = [s.draw(rnd) for s in arg_strategies]
                    keyword = {k: s.draw(rnd) for k, s in kwarg_strategies.items()}
                except _UnsatisfiedAssumption:
                    continue
                try:
                    func(*args, *positional, **kwargs, **keyword)
                except _UnsatisfiedAssumption:
                    continue
                except AssertionError as exc:
                    raise AssertionError(
                        f"Falsifying example for {func.__qualname__}: "
                        f"args={positional!r} kwargs={keyword!r}\n{exc}"
                    ) from exc
                drawn += 1

            if drawn == 0:
                raise AssertionError(
                    f"{func.__qualname__}: no example satisfied the assumptions")
        wrapper._shim_given = True
        return wrapper
    return decorator


def example(*args, **kwargs):
    """Accepted and ignored: explicit examples are run by the wrapped test."""
    def decorator(func):
        return func
    return decorator


class HealthCheck:
    too_slow = "too_slow"
    filter_too_much = "filter_too_much"
    data_too_large = "data_too_large"
    large_base_example = "large_base_example"
    function_scoped_fixture = "function_scoped_fixture"


Phase = type("Phase", (), {"explicit": 0, "reuse": 1, "generate": 2,
                           "target": 3, "shrink": 4})
