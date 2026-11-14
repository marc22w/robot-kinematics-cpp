"""Genereaza graficele pentru README din fisierele CSV scrise de ./build/demo.

Rulare (din radacina proiectului):  python scripts/grafice.py
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DOCS = Path("docs")
CULORI = ["#4C9BE8", "#FF6B35", "#2CA58D", "#E4572E", "#9B5DE5", "#F2C14E"]
VMAX = 180.0                 # grade/s, aceeasi limita ca in demo
AMAX = np.degrees(4.0)       # 4 rad/s^2 in grade/s^2

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3,
                     "axes.spines.top": False, "axes.spines.right": False})


def citeste(fisier):
    return np.genfromtxt(fisier, delimiter=",", names=True)


def profil_ptp():
    d = citeste("traiectorie_ptp.csv")
    t = d["t"]
    fig, axe = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for i in range(6):
        eticheta = f"A{i + 1}"
        axe[0].plot(t, np.degrees(d[f"q{i + 1}"]), color=CULORI[i], label=eticheta)
        axe[1].plot(t, np.degrees(d[f"qd{i + 1}"]), color=CULORI[i])
        axe[2].plot(t, np.degrees(d[f"qdd{i + 1}"]), color=CULORI[i])
    for ax, limita in ((axe[1], VMAX), (axe[2], AMAX)):
        for semn in (1, -1):
            ax.axhline(semn * limita, color="black", ls="--", lw=1)
        ax.text(t[-1], limita, " limita", va="bottom", ha="right", fontsize=8)
    axe[0].set_ylabel("Unghi (°)")
    axe[1].set_ylabel("Viteza (°/s)")
    axe[2].set_ylabel("Acceleratie (°/s²)")
    axe[2].set_xlabel("Timp (s)")
    axe[0].legend(ncol=6, loc="upper center", bbox_to_anchor=(0.5, 1.25), frameon=False)
    fig.suptitle(f"Miscare PTP la durata minima ({t[-1]:.2f} s): pornire si oprire fara smucituri",
                 y=1.0)
    fig.tight_layout()
    fig.savefig(DOCS / "ptp_profil.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def abatere_maxima(p):
    a, b = p[0], p[-1]
    d = (b - a) / np.linalg.norm(b - a)
    rel = p - a
    return np.linalg.norm(rel - np.outer(rel @ d, d), axis=1).max()


def ptp_vs_lin():
    ptp, lin = citeste("cale_ptp.csv"), citeste("cale_lin.csv")
    P = np.column_stack([ptp["x"], ptp["y"], ptp["z"]])
    L = np.column_stack([lin["x"], lin["y"], lin["z"]])
    abatere = abatere_maxima(P)

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(projection="3d")
    ax.plot(*P.T, color=CULORI[1], lw=2.5, label=f"PTP (articulatii): abatere max. {abatere:.1f} mm")
    ax.plot(*L.T, color=CULORI[0], lw=2.5, label="LIN (cartezian): linie dreapta")
    ax.scatter(*P[0], color="black", s=50, zorder=5)
    ax.scatter(*P[-1], color="black", s=50, marker="s", zorder=5)
    ax.text(*P[0], "  start", fontsize=9)
    ax.text(*P[-1], "  final", fontsize=9)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("z (mm)")
    ax.view_init(elev=25, azim=-60)
    ax.legend(loc="upper left")
    ax.set_title("Acelasi punct final, doua tipuri de miscare: traseul sculei")
    fig.tight_layout()
    fig.savefig(DOCS / "ptp_vs_lin.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    return abatere


if __name__ == "__main__":
    DOCS.mkdir(exist_ok=True)
    profil_ptp()
    abatere = ptp_vs_lin()
    print("Salvat: docs/ptp_profil.png, docs/ptp_vs_lin.png")
    print(f"Abaterea maxima a miscarii PTP fata de linia dreapta: {abatere:.1f} mm")
