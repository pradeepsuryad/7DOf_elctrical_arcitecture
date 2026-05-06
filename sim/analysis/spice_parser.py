"""
Parse ngspice ASCII output files produced by the `wrdata` command.

ngspice wrdata format (no header — raw paired columns):
  Each row: t1 v1 t2 v2 t3 v3 ...
  where t1==t2==t3==... (same time scale per row, one pair per signal).

Signal order matches the wrdata argument order in the .cir file:
  v(BUS_OUT)  v(SENSE_LO)  v(GATE)  I(Iload)

Returns a dict: signal name -> 1-D numpy array, plus 'time'.

Example
-------
    from spice_parser import load_wrdata, find_trip_time
    data = load_wrdata('sim/spice/spice_output.dat')
    time  = data['time']
    gate  = data['v(gate)']
    trip  = find_trip_time(time, gate)
"""

from pathlib import Path
from typing import Dict
import numpy as np

# Signal names matching the wrdata line in overcurrent_protection.cir:
#   wrdata spice_output.dat v(BUS_OUT) v(SENSE_LO) v(GATE) I(Iload)
SIGNAL_NAMES = ['v(bus_out)', 'v(sense_lo)', 'v(gate)', 'i(iload)']


def load_wrdata(path: str | Path) -> Dict[str, np.ndarray]:
    """
    Load an ngspice wrdata ASCII file.

    ngspice wrdata writes one time+value pair per signal per row:
        t  sig0  t  sig1  t  sig2 ...
    This function extracts the common time axis (column 0) and each signal.

    Returns
    -------
    dict with keys 'time' and each name from SIGNAL_NAMES (all lowercase).
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"SPICE output not found: {path}\n"
            "Run: ngspice_con -b sim/spice/overcurrent_protection.cir")

    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('*'):
            continue
        try:
            vals = [float(x) for x in line.split()]
            if vals:
                rows.append(vals)
        except ValueError:
            continue

    if not rows:
        raise ValueError(f"No numeric data in {path}")

    arr = np.array(rows)   # shape: (n_points, 2 * n_signals)

    # Column layout: t sig0 t sig1 t sig2 ...
    time    = arr[:, 0]
    signals = {name: arr[:, 1 + 2 * i] for i, name in enumerate(SIGNAL_NAMES)}
    signals['time'] = time
    return signals


def find_trip_time(time: np.ndarray, gate_v: np.ndarray,
                   vth: float = 4.0, fault_onset_s: float = 50e-6) -> float:
    """
    Return the time (s) when gate voltage first drops below vth after fault onset.
    Returns NaN if the gate never trips.
    """
    mask    = time >= fault_onset_s
    t_after = time[mask]
    vg      = gate_v[mask]
    tripped = np.where(vg < vth)[0]
    if len(tripped) == 0:
        return float('nan')
    return float(t_after[tripped[0]])
