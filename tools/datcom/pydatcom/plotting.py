"""Plot DATCOM aerodynamic coefficients (optional, requires matplotlib).

Python port of MATLAB ``plot_aero.m``. Produces:
  * static coefficients vs angle of attack (CN, CD, CM, Xcp),
  * key derivatives vs Mach (CNa, CMa, CD0),
  * fin-control effectiveness when control data is present.
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .importer import AeroData


def _closest(v: np.ndarray, t: float) -> int:
    return int(np.argmin(np.abs(v - t)))


def plot_aero(aero: AeroData, mach_show: Optional[Sequence[float]] = None):
    """Create the coefficient figures and return the list of figures."""
    import matplotlib.pyplot as plt

    mach = np.asarray(aero.mach).ravel()
    alpha = np.asarray(aero.alpha).ravel()

    if mach_show is None or len(mach_show) == 0:
        mach_show = [0.1, 0.3, 0.6, 1.8, 3.0, 5.0]
    idx = list(dict.fromkeys(_closest(mach, t) for t in mach_show))
    labels = [f"M = {mach[i]:.2f}" for i in idx]
    cmap = plt.get_cmap("tab10")
    colors = [cmap(k % 10) for k in range(len(idx))]

    figs = []

    # --- Figure 1: static coefficients vs alpha ---
    fig1, axes = plt.subplots(2, 2, figsize=(10, 7))
    fig1.canvas.manager.set_window_title("Static coefficients vs alpha")
    panels = [(axes[0, 0], aero.cn, "C_N", "Normal force", False),
              (axes[0, 1], aero.cd, "C_D", "Drag", False),
              (axes[1, 0], aero.cm, "C_M", "Pitching moment", False),
              (axes[1, 1], aero.xcp, "X_cp (aft of CG)", "Centre of pressure", True)]
    for ax, Y, yl, ttl, clip in panels:
        for k, i in enumerate(idx):
            ax.plot(alpha, Y[:, i], lw=1.4, color=colors[k])
        ax.grid(True); ax.set_xlabel(r"$\alpha$ [deg]")
        ax.set_ylabel(yl); ax.set_title(ttl)
        if clip:
            vals = Y[:, idx]
            vals = vals[np.isfinite(vals)]
            if vals.size:
                lo, hi = np.percentile(vals, 2), np.percentile(vals, 98)
                if hi > lo:
                    pad = 0.1 * (hi - lo)
                    ax.set_ylim(lo - pad, hi + pad)
    axes[1, 1].legend(labels, loc="best")
    fig1.suptitle("Static aerodynamic coefficients")
    fig1.tight_layout()
    figs.append(fig1)

    # --- Figure 2: derivatives vs Mach (near alpha = 0) ---
    ia0 = _closest(alpha, 0.0)
    fig2, ax2 = plt.subplots(1, 3, figsize=(12, 4))
    fig2.canvas.manager.set_window_title("Derivatives vs Mach")
    ax2[0].plot(mach, aero.cla[ia0, :], "-o", lw=1.4, color=colors[0])
    ax2[0].set_xlabel("Mach"); ax2[0].set_ylabel(r"$C_{N\alpha}$ [1/deg]")
    ax2[0].set_title("Normal-force slope")
    ax2[1].plot(mach, aero.cma[ia0, :], "-o", lw=1.4, color=colors[min(1, len(colors) - 1)])
    ax2[1].set_xlabel("Mach"); ax2[1].set_ylabel(r"$C_{M\alpha}$ [1/deg]")
    ax2[1].set_title("Pitch stability")
    ax2[2].plot(mach, aero.cd[ia0, :], "-o", lw=1.4, color=colors[min(2, len(colors) - 1)])
    ax2[2].set_xlabel("Mach"); ax2[2].set_ylabel(r"$C_{D0}$")
    ax2[2].set_title(r"Zero-$\alpha$ drag")
    for ax in ax2:
        ax.grid(True)
    fig2.suptitle(r"Aerodynamic derivatives vs Mach ($\alpha \approx 0$)")
    fig2.tight_layout()
    figs.append(fig2)

    # --- Figure 3: fin control effectiveness (if present) ---
    if aero.ndelta and aero.ndelta > 0:
        fig3, ax3 = plt.subplots(1, 3, figsize=(12, 4))
        fig3.canvas.manager.set_window_title("Fin control effectiveness")
        delta = np.asarray(aero.delta).ravel()
        for k, i in enumerate(idx):
            ax3[0].plot(delta, aero.dcl_sym[:, i], "-o", color=colors[k], lw=1.4)
            ax3[1].plot(delta, aero.dcm_sym[:, i], "-o", color=colors[k], lw=1.4)
        ax3[0].set_xlabel(r"$\delta$ [deg]"); ax3[0].set_ylabel(r"$\Delta C_L$")
        ax3[0].set_title("Pitch: lift increment"); ax3[0].legend(labels, loc="best")
        ax3[1].set_xlabel(r"$\delta$ [deg]"); ax3[1].set_ylabel(r"$\Delta C_M$")
        ax3[1].set_title("Pitch: moment increment")
        if len(aero.deltal) > 0:
            dl = np.asarray(aero.deltal).ravel()
            for k, i in enumerate(idx):
                ax3[2].plot(dl, aero.clroll[:, i], "-o", color=colors[k], lw=1.4)
            ax3[2].set_xlabel(r"$\delta_L$ [deg]"); ax3[2].set_ylabel(r"$(C_l)_{roll}$")
            ax3[2].set_title("Roll effectiveness")
        for ax in ax3:
            ax.grid(True)
        fig3.suptitle("Fin control effectiveness")
        fig3.tight_layout()
        figs.append(fig3)

    return figs
