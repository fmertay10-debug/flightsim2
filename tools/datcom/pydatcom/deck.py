"""DATCOM input-deck primitives: namelist cards, control cards, assembly.

Encodes the Digital DATCOM card rules (Users Manual, Section 3 and
Appendix A) in one place:

* namelist cards start in column 2 (`` $NAME``), control cards in column 1;
* values are comma-separated; arrays use ``VAR(1)=v1,v2,...``;
* every numeric constant carries a decimal point;
* a card is terminated by ``$``;
* continuation lines are indented two spaces. DATCOM's ``for006.dat`` echo
  prepends one more space, and :mod:`pydatcom.importer` re-reads the card
  echo relying on that three-space continuation indent.

Two further constraints exist because the importer parses the *echo* of
these cards (see ``import_datcom``):

* a variable the importer searches for on the card's first line
  (``MACH(1)=``, ``ALSCHD(1)=``, ``XCG=``/``ZCG=``, ``SREF=``...) must be
  emitted on that first line — pass ``nl=False`` (the default);
* ``DELTAL(1)=`` / ``DELTAR(1)=`` must each *start* a fresh line
  (``nl=True``) so the importer's per-key scan sees them.
"""
from __future__ import annotations

from typing import List, Sequence, Union

_MAX_LINE = 75          # wrap before DATCOM's 80-column card limit
_ARRAY_PER_LINE = 6     # array values per line (readability + echo parsing)


class Namelist:
    """One namelist card, e.g. ``$FLTCON NMACH=17.0,MACH(1)=...$``.

    Items are emitted in insertion order. All ``add`` methods return
    ``self`` for chaining.
    """

    def __init__(self, name: str):
        self.name = name
        # each item: (nl_flag, [token, ...], per_line)
        self._items: List[tuple] = []

    def num(self, var: str, value: float, fmt: str = "%.2f",
            nl: bool = False) -> "Namelist":
        """Scalar numeric variable (formatted with a decimal point)."""
        self._items.append((nl, [f"{var}={fmt % value}"], 1))
        return self

    def lit(self, var: str, text: str, nl: bool = False) -> "Namelist":
        """Literal (pre-formatted) value, e.g. ``VERTUP`` -> ``.TRUE.``."""
        self._items.append((nl, [f"{var}={text}"], 1))
        return self

    def arr(self, var: str, values: Sequence[float], fmt: str = "%.2f",
            nl: bool = False, per_line: int = _ARRAY_PER_LINE) -> "Namelist":
        """Array variable ``VAR(1)=v1,v2,...`` wrapped ``per_line`` per line."""
        toks = [fmt % v for v in values]
        toks[0] = f"{var}(1)={toks[0]}"
        self._items.append((nl, toks, per_line))
        return self

    def render(self) -> str:
        if not self._items:
            raise ValueError(f"namelist ${self.name} has no variables")
        # Flatten to (token, break_before) honoring nl flags and array wrap
        flat: List[tuple] = []
        for nl, toks, per_line in self._items:
            for k, tok in enumerate(toks):
                brk = (nl and k == 0 and bool(flat)) or (k > 0 and k % per_line == 0)
                flat.append((tok, brk))

        lines: List[str] = []
        cur = f" ${self.name} {flat[0][0]}"
        for tok, brk in flat[1:]:
            if brk or len(cur) + 1 + len(tok) > _MAX_LINE:
                lines.append(cur + ",")
                cur = "  " + tok
            else:
                cur += "," + tok
        lines.append(cur + "$")
        return "\n".join(lines) + "\n"


class Deck:
    """An ordered sequence of namelist cards and control cards."""

    def __init__(self):
        self._parts: List[str] = []

    def card(self, namelist: Namelist) -> "Deck":
        self._parts.append(namelist.render())
        return self

    def control(self, text: str) -> "Deck":
        """Control card in column 1 (CASEID, DIM, DAMP, SAVE, NACA-, ...)."""
        self._parts.append(text + "\n")
        return self

    def text(self) -> str:
        return "".join(self._parts)

    def write(self, path: str) -> str:
        with open(path, "w") as f:
            f.write(self.text())
        return path
