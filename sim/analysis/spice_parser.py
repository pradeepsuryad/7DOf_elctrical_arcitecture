"""
Parse ngspice ASCII output files produced by the `wrdata` command.

ngspice wrdata format (two header lines, then space-separated columns):
  Line 1: signal names separated by spaces  (e.g. "time v(sense_hi) v(gate) ...")
  Line 2: blank or units line (skipped)
  Rest:   rows of floating-point numbers, one row per time step

Returns a dict mapping lowercase signal name → 1-D numpy array.

Example
-------
    from spice_parser import load_wrdata
    data = load_wrdata('sim/spice/spice_output.dat')
    time        = data['time']
    gate_volts  = data['v(gate)']
"""

from pathlib import Path
from typing import Dict
import numpy as np


def load_wrdata(path: str | Path) -> Dict[str, np.ndarray]:
    """
    Load an ngspice `wrdata` ASCII file.

    Parameters
    ----------
    path : path to the .dat file produced by ngspice wrdata

    Returns
    -------
    dict of {signal_name: np.ndarray}  — all names are lowercased.

    Raises
    ------
    FileNotFoundError if the file does not exist.
    ValueError if the file cannot be parsed.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(
            f"SPICE output not found: {path}\n"
            "Run the simulation first:\n"
            "  ngspice -b sim/spice/overcurrent_protection.cir\n"
            "The output file lands next to the .cir file.")

    lines = path.read_text().splitlines()

    # Find the header line (first non-empty, non-comment line).
    header_idx = next(
        (i for i, ln in enumerate(lines) if ln.strip() and not ln.startswith('*')),
        None)
    if header_idx is None:
        raise ValueError(f"Could not find header in {path}")

    names = lines[header_idx].split()
    names = [n.lower() for n in names]

    # Skip the header line (and an optional units line directly after it).
    data_start = header_idx + 1
    if data_start < len(lines) and not _is_data_line(lines[data_start]):
        data_start += 1

    # Parse numeric rows.
    rows = []
    for ln in lines[data_start:]:
        ln = ln.strip()
        if not ln or ln.startswith('*'):
            continue
        try:
            rows.append([float(x) for x in ln.split()])
        except ValueError:
            continue  # skip malformed lines (e.g. blank/comment inside data)

    if not rows:
        raise ValueError(f"No numeric data found in {path}")

    arr = np.array(rows)  # shape: (n_points, n_signals)

    if arr.shape[1] != len(names):
        raise ValueError(
            f"Header has {len(names)} columns but data has {arr.shape[1]}. "
            "Check wrdata signal list in the .cir file.")

    return {name: arr[:, i] for i, name in enumerate(names)}


def _is_data_line(line: str) -> bool:
    """True if the line looks like a row of numbers (not a header or comment)."""
    line = line.strip()
    if not line or line.startswith('*'):
        return False
    try:
        float(line.split()[0])
        return True
    except (ValueError, IndexError):
        return False


def find_trip_time(time: np.ndarray, gate_v: np.ndarray,
                   vth: float = 4.0, fault_onset_s: float = 50e-6) -> float:
    """
    Return the time (seconds) at which the gate voltage first drops below `vth`
    after `fault_onset_s`.  Returns NaN if the gate never trips.

    Parameters
    ----------
    time        : time array from load_wrdata
    gate_v      : V(GATE) array from load_wrdata
    vth         : MOSFET gate threshold voltage (V), default 4.0 V
    fault_onset_s : simulation time when the fault was injected
    """
    mask = time >= fault_onset_s
    t_after  = time[mask]
    vg_after = gate_v[mask]

    tripped = np.where(vg_after < vth)[0]
    if len(tripped) == 0:
        return float('nan')
    return float(t_after[tripped[0]])
