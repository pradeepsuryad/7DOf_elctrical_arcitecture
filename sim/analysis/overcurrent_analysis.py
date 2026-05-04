"""
Overcurrent protection circuit — analytical model and design sweep plots.

Analyses the bus protection stage in sim/spice/overcurrent_protection.cir.
Runs entirely from first principles — no SPICE installation needed.
If a spice_output.dat file exists (from ngspice), overlays the simulation
waveforms on the analytical plots for cross-validation.

Outputs (saved to sim/analysis/figures/):
  01_trip_current_vs_rsense.png   Design curve: I_trip and P_cont vs R_sense
  02_gate_fall_time.png           Gate voltage waveform + analytical prediction
  03_protection_timeline.png      Full fault event: sense V, gate V, bus V

Usage:
  python overcurrent_analysis.py                        # analytical only
  python overcurrent_analysis.py --spice spice_out.dat  # + simulation overlay

Dependencies:  numpy  matplotlib
"""

import argparse
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')   # non-interactive backend — safe on headless / CI
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

try:
    from spice_parser import load_wrdata, find_trip_time
    PARSER_AVAILABLE = True
except ImportError:
    PARSER_AVAILABLE = False


# ── Circuit parameters (matches overcurrent_protection.cir) ──────────────────

V_BUS       = 48.0      # V
V_REF       = 75e-3     # V  — comparator trip threshold
R_SENSE     = 5e-3      # Ω  — nominal design point
I_CONT      = 10.0      # A  — normal continuous current
I_FAULT     = 25.0      # A  — fault current in simulation
FAULT_T     = 50e-6     # s  — fault onset time in simulation

V_GATE_INIT = 12.0      # V  — gate supply
V_TH        = 4.0       # V  — MOSFET gate threshold
R_PULLUP    = 10e3      # Ω
R_COMP      = 100.0     # Ω  — comparator output resistor
R_GATE      = 10.0      # Ω  — series gate resistor
C_GS        = 3.5e-9    # F  — MOSFET gate-source capacitance

T_COMP_PROP = 150e-9    # s  — LM393 propagation delay

FIGURES_DIR = Path(__file__).parent / 'figures'


# ── Analytical models ─────────────────────────────────────────────────────────

def trip_current(r_sense: float, v_ref: float = V_REF) -> float:
    """Current at which the comparator trips (A)."""
    return v_ref / r_sense


def sense_power(i_cont: float, r_sense: float) -> float:
    """Continuous power dissipation in the sense resistor (W)."""
    return i_cont ** 2 * r_sense


def gate_rc_fall_time(r_pull: float = R_PULLUP,
                      r_comp: float = R_COMP,
                      r_gate: float = R_GATE,
                      c_gs:   float = C_GS,
                      v_init: float = V_GATE_INIT,
                      v_th:   float = V_TH) -> float:
    """
    Time (s) for gate to fall from v_init to v_th after comparator trips.

    When the comparator output goes LOW (0 V), the gate sees:
      - Pull-up source:  r_pull to V_GATE_INIT
      - Pull-down sink:  r_comp + r_gate to GND

    Thevenin equivalent at gate node:
      V_th_eq = V_GATE_INIT × (r_comp + r_gate) / (r_pull + r_comp + r_gate)
                ≈ 0 V  (since r_pull >> r_comp + r_gate)
      R_th_eq = r_pull || (r_comp + r_gate)  ≈ r_comp + r_gate

    Gate decays: V_gate(t) = V_th_eq + (v_init - V_th_eq) × exp(-t / τ)
    with τ = R_th_eq × c_gs.

    Solving for V_gate(t_fall) = v_th:
      t_fall = τ × ln((v_init - V_th_eq) / (v_th - V_th_eq))
    """
    r_th  = (r_pull * (r_comp + r_gate)) / (r_pull + r_comp + r_gate)
    v_th_eq = V_GATE_INIT * (r_comp + r_gate) / (r_pull + r_comp + r_gate)
    tau   = r_th * c_gs

    if v_th <= v_th_eq:
        return float('inf')  # gate never reaches threshold with these values

    t_fall = tau * math.log((v_init - v_th_eq) / (v_th - v_th_eq))
    return t_fall


def total_trip_time(r_sense: float = R_SENSE) -> float:
    """
    Total time from fault onset to FET off (s).
      = comparator propagation delay + gate fall time
    Note: trip time is independent of R_sense in this model (R_sense only
    affects the threshold voltage, not the gate drive RC).
    """
    return T_COMP_PROP + gate_rc_fall_time()


def gate_voltage_waveform(t: np.ndarray, fault_onset: float = FAULT_T
                          ) -> np.ndarray:
    """
    Analytical gate voltage vs time array.
    Before fault: 12 V (FET on).
    After fault onset + comparator delay: RC decay toward 0 V.
    """
    v = np.full_like(t, V_GATE_INIT)
    t_trip = fault_onset + T_COMP_PROP
    mask   = t >= t_trip

    r_th   = (R_PULLUP * (R_COMP + R_GATE)) / (R_PULLUP + R_COMP + R_GATE)
    v_ss   = V_GATE_INIT * (R_COMP + R_GATE) / (R_PULLUP + R_COMP + R_GATE)
    tau    = r_th * C_GS

    v[mask] = v_ss + (V_GATE_INIT - v_ss) * np.exp(-(t[mask] - t_trip) / tau)
    return v


# ── Plot 1 — Trip current and sense power vs R_sense ─────────────────────────

def plot_design_sweep(save_dir: Path) -> None:
    """
    Design tradeoff curve: as R_sense increases:
      - trip current decreases (tighter protection)
      - continuous power dissipation increases (more heat at normal load)

    The vertical line shows the nominal 5 mΩ design point.
    The horizontal band shows the 2 W resistor rating.
    """
    r_range  = np.linspace(1e-3, 30e-3, 500)
    i_trip   = trip_current(r_range)
    p_cont   = sense_power(I_CONT, r_range)

    fig, ax1 = plt.subplots(figsize=(8, 5))
    ax2 = ax1.twinx()

    ax1.plot(r_range * 1e3, i_trip,  color='#1f77b4', lw=2, label='Trip current (A)')
    ax2.plot(r_range * 1e3, p_cont,  color='#d62728', lw=2, ls='--',
             label=f'Continuous power at {I_CONT:.0f} A (W)')

    # Nominal design point
    ax1.axvline(R_SENSE * 1e3, color='gray', ls=':', lw=1.5, label='Nominal 5 mΩ')
    ax1.axhline(trip_current(R_SENSE), color='#1f77b4', ls=':', lw=1, alpha=0.5)
    ax2.axhline(2.0, color='#d62728', ls=':', lw=1, alpha=0.5,
                label='2 W resistor rating')

    # Annotations at nominal point
    ax1.annotate(
        f'  I_trip = {trip_current(R_SENSE):.0f} A\n  P = {sense_power(I_CONT, R_SENSE)*1000:.0f} mW',
        xy=(R_SENSE * 1e3, trip_current(R_SENSE)),
        xytext=(R_SENSE * 1e3 + 3, trip_current(R_SENSE) + 2),
        fontsize=9, color='#1f77b4')

    ax1.set_xlabel('R_sense (mΩ)')
    ax1.set_ylabel('Trip current (A)', color='#1f77b4')
    ax2.set_ylabel('Sense resistor power at 10 A (W)', color='#d62728')
    ax1.set_title('Overcurrent Protection — Design Sweep\nTrip current vs sense resistor value')

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=9)

    ax1.grid(True, alpha=0.3)
    fig.tight_layout()
    out = save_dir / '01_trip_current_vs_rsense.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ── Plot 2 — Gate voltage waveform ────────────────────────────────────────────

def plot_gate_waveform(save_dir: Path, spice_data: dict = None) -> None:
    """
    Gate voltage from fault onset to full turn-off.
    Shows: analytical RC decay, VTH threshold, and (if available) SPICE simulation.
    """
    t_end  = FAULT_T + 3e-6    # 3 µs after fault — captures full decay
    t      = np.linspace(FAULT_T - 0.5e-6, t_end, 5000)
    v_gate = gate_voltage_waveform(t)

    t_fall  = gate_rc_fall_time()
    t_total = total_trip_time()

    fig, ax = plt.subplots(figsize=(8, 4.5))

    ax.plot((t - FAULT_T) * 1e6, v_gate,
            color='#1f77b4', lw=2, label='Analytical model')

    if spice_data and 'v(gate)' in spice_data and 'time' in spice_data:
        t_s = spice_data['time']
        v_s = spice_data['v(gate)']
        mask = (t_s >= FAULT_T - 0.5e-6) & (t_s <= t_end)
        ax.plot((t_s[mask] - FAULT_T) * 1e6, v_s[mask],
                color='#ff7f0e', lw=1.5, ls='--', label='ngspice simulation')

    ax.axhline(V_TH, color='red', ls=':', lw=1.5,
               label=f'V_th = {V_TH:.0f} V (FET threshold)')
    ax.axvline(0, color='gray', ls='--', lw=1, label='Fault onset')
    ax.axvline(T_COMP_PROP * 1e6, color='purple', ls=':', lw=1,
               label=f'Comparator delay = {T_COMP_PROP*1e9:.0f} ns')
    ax.axvline(t_total * 1e6, color='green', ls=':', lw=1,
               label=f'FET off at {t_total*1e9:.0f} ns')

    ax.fill_betweenx([0, V_GATE_INIT],
                     T_COMP_PROP * 1e6, t_total * 1e6,
                     alpha=0.08, color='orange', label='Gate RC fall')

    ax.set_xlabel('Time after fault onset (µs)')
    ax.set_ylabel('Gate voltage (V)')
    ax.set_title(f'Gate Turn-Off Waveform\n'
                 f'τ = {gate_rc_fall_time()*1e9:.0f} ns  |  '
                 f'Total trip time = {t_total*1e9:.0f} ns')
    ax.set_ylim(-0.5, V_GATE_INIT + 1)
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = save_dir / '02_gate_fall_time.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ── Plot 3 — Full fault event timeline ───────────────────────────────────────

def plot_fault_timeline(save_dir: Path, spice_data: dict = None) -> None:
    """
    Three-panel view of the full simulation window:
      Top:    Sense resistor voltage (= current × R_sense)
      Middle: Gate voltage
      Bottom: Bus output voltage
    """
    t = np.linspace(0, 500e-6, 50000)

    # Analytical sense voltage: step from 10 A to 25 A at FAULT_T
    v_sense = np.where(t < FAULT_T,
                       I_CONT  * R_SENSE,
                       I_FAULT * R_SENSE)

    v_gate = gate_voltage_waveform(t)

    # Bus voltage: held by Cbus until FET opens, then drops as Iload discharges it
    t_trip  = total_trip_time() + FAULT_T
    c_bus   = 100e-6
    v_bus   = np.where(t < t_trip,
                       V_BUS,
                       V_BUS - I_FAULT * (t - t_trip) / c_bus)
    v_bus   = np.clip(v_bus, 0, V_BUS)

    fig = plt.figure(figsize=(10, 7))
    gs  = gridspec.GridSpec(3, 1, hspace=0.05, figure=fig)
    axes = [fig.add_subplot(gs[i]) for i in range(3)]

    colors = ['#2ca02c', '#1f77b4', '#d62728']
    labels = ['V(sense_hi) — mV', 'V(gate) — V', 'V(bus_out) — V']
    data_an = [v_sense * 1e3, v_gate, v_bus]

    for ax, col, lbl, d_an in zip(axes, colors, labels, data_an):
        ax.plot(t * 1e6, d_an, color=col, lw=1.5, label='Analytical')
        ax.set_ylabel(lbl, fontsize=9)
        ax.axvline(FAULT_T * 1e6, color='gray', ls='--', lw=1, alpha=0.6)
        ax.grid(True, alpha=0.3)
        ax.tick_params(axis='x', labelbottom=False)

    # SPICE overlay
    spice_keys = ['v(sense_hi)', 'v(gate)', 'v(48v_out)']
    if spice_data:
        for ax, sk in zip(axes, spice_keys):
            if sk in spice_data and 'time' in spice_data:
                ax.plot(spice_data['time'] * 1e6,
                        spice_data[sk] * (1e3 if sk == 'v(sense_hi)' else 1),
                        color='orange', lw=1, ls='--', alpha=0.8,
                        label='ngspice')

    # Annotations on top panel
    axes[0].axhline(V_REF * 1e3, color='red', ls=':', lw=1,
                    label=f'V_ref = {V_REF*1e3:.0f} mV (trip)')
    axes[0].annotate('Fault!', xy=(FAULT_T * 1e6, I_FAULT * R_SENSE * 1e3),
                     xytext=(FAULT_T * 1e6 + 10, I_FAULT * R_SENSE * 1e3 * 0.85),
                     fontsize=9, color='red',
                     arrowprops=dict(arrowstyle='->', color='red'))

    for ax in axes:
        ax.legend(fontsize=8, loc='upper left')

    axes[-1].tick_params(axis='x', labelbottom=True)
    axes[-1].set_xlabel('Time (µs)')
    axes[0].set_title('Overcurrent Protection — Fault Event Timeline\n'
                       f'Bus = {V_BUS:.0f} V  |  '
                       f'I_fault = {I_FAULT:.0f} A  |  '
                       f'R_sense = {R_SENSE*1e3:.0f} mΩ  |  '
                       f'V_ref = {V_REF*1e3:.0f} mV')

    out = save_dir / '03_protection_timeline.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"  Saved: {out.name}")


# ── Design summary ────────────────────────────────────────────────────────────

def print_design_summary() -> None:
    t_fall  = gate_rc_fall_time()
    t_total = total_trip_time()

    r_th = (R_PULLUP * (R_COMP + R_GATE)) / (R_PULLUP + R_COMP + R_GATE)
    tau  = r_th * C_GS

    print("\n" + "═" * 56)
    print("  Overcurrent Protection — Design Summary")
    print("═" * 56)
    print(f"  Bus voltage            {V_BUS:.0f} V")
    print(f"  Continuous current     {I_CONT:.0f} A")
    print(f"  Sense resistor         {R_SENSE*1e3:.1f} mΩ")
    print(f"  V_sense at I_cont      {sense_power(I_CONT, R_SENSE)/I_CONT*1e3:.1f} mV"
          f"  (= {I_CONT:.0f} A × {R_SENSE*1e3:.0f} mΩ)")
    print(f"  V_ref (trip threshold) {V_REF*1e3:.0f} mV")
    print(f"  Trip current           {trip_current(R_SENSE):.0f} A")
    print(f"  Headroom               {trip_current(R_SENSE)/I_CONT:.1f}× continuous")
    print()
    print(f"  Sense resistor power @ {I_CONT:.0f} A:  "
          f"{sense_power(I_CONT, R_SENSE)*1e3:.0f} mW")
    print(f"  Sense resistor power @ {trip_current(R_SENSE):.0f} A:  "
          f"{sense_power(trip_current(R_SENSE), R_SENSE)*1000:.0f} mW")
    print()
    print(f"  Gate drive Thevenin R  {r_th:.1f} Ω")
    print(f"  Gate RC time constant  {tau*1e9:.0f} ns")
    print(f"  Gate fall time         {t_fall*1e9:.0f} ns  "
          f"(12 V → {V_TH:.0f} V threshold)")
    print(f"  Comparator prop. delay {T_COMP_PROP*1e9:.0f} ns  (LM393 typ)")
    print(f"  Total trip time        {t_total*1e9:.0f} ns")
    print(f"  Energy in Rsense       "
          f"{(I_FAULT**2 * R_SENSE * t_total)*1e6:.2f} µJ  "
          f"(fault burst, negligible)")
    print("═" * 56 + "\n")


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyse the overcurrent protection circuit analytically.")
    parser.add_argument(
        '--spice', metavar='PATH', default=None,
        help='Path to ngspice wrdata output file for simulation overlay.')
    args = parser.parse_args()

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)

    spice_data = None
    if args.spice:
        if not PARSER_AVAILABLE:
            print("Warning: spice_parser.py not found — skipping simulation overlay.")
        else:
            try:
                spice_data = load_wrdata(args.spice)
                print(f"Loaded SPICE data: {list(spice_data.keys())}")
                t_trip_sim = find_trip_time(
                    spice_data['time'], spice_data.get('v(gate)', np.array([])))
                if not math.isnan(t_trip_sim):
                    print(f"  Simulated trip time: "
                          f"{(t_trip_sim - FAULT_T)*1e9:.0f} ns after fault")
            except Exception as e:
                print(f"Warning: could not load SPICE data: {e}")

    print_design_summary()

    print("Generating plots...")
    plot_design_sweep(FIGURES_DIR)
    plot_gate_waveform(FIGURES_DIR, spice_data)
    plot_fault_timeline(FIGURES_DIR, spice_data)
    print(f"\nAll figures saved to: {FIGURES_DIR}/")


if __name__ == '__main__':
    main()
