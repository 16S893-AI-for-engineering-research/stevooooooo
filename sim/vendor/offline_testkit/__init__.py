"""Minimal stand-ins for pytest and hypothesis, for offline use only.

Why this exists. The sandbox this was built in has no PyPI access:

    $ curl https://pypi.org/simple/pytest/
    curl: (56) Received HTTP code 403 from proxy after CONNECT

so ``uv add --dev pytest hypothesis`` cannot resolve. Rather than ship an
unrunnable test suite, this package provides just enough of both APIs to
execute the real tests unmodified: ``@given``, the strategies used,
``@settings``, ``assume``, ``pytest.approx``, ``pytest.raises``,
``@pytest.mark.parametrize``, module-scoped fixtures and ``pytest.fail``.

It is NOT a replacement for either library. In particular hypothesis's
shrinking, database, deadline handling and coverage-guided generation are all
absent -- this simply samples each strategy at random, which catches far less.

When the network is available, prefer the real thing:

    uv sync --extra dev
    uv run pytest

``tools/run_tests.py`` uses the real libraries if it can import them and falls
back to these shims otherwise, printing which mode it chose.
"""

from __future__ import annotations

__all__ = ["install"]


def install() -> None:
    """Register the shim modules in ``sys.modules`` if the real ones are absent."""
    import sys

    try:
        import pytest  # noqa: F401
        import hypothesis  # noqa: F401
        return
    except ImportError:
        pass

    from . import _hypothesis, _pytest

    # Bind under the canonical names so `from hypothesis import given` and
    # `from hypothesis.strategies import floats` both resolve. Assign rather
    # than setdefault: a partially-initialised real package may already be
    # present in sys.modules after a failed import.
    sys.modules["pytest"] = _pytest
    sys.modules["hypothesis"] = _hypothesis
    sys.modules["hypothesis.strategies"] = _hypothesis.strategies
