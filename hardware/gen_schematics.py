#!/usr/bin/env python3
"""
Generate valid KiCad 7 S-expression schematics for all three arm-core boards.
Run from repo root:  python hardware/gen_schematics.py
"""
import os

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)))


# ── UUID helpers ─────────────────────────────────────────────────────────────

def u(n: int) -> str:
    h = f"{n:032x}"
    return f"{h[0:8]}-{h[8:12]}-{h[12:16]}-{h[16:20]}-{h[20:32]}"


# ── Low-level KiCad 7 S-expression builders ──────────────────────────────────

def _prop(name, value, x, y, hide=False, size=1.27, justify="left"):
    h = " (hide yes)" if hide else ""
    j = f" (justify {justify})" if not hide else ""
    return (f'    (property "{name}" "{value}" (at {x} {y} 0)\n'
            f'      (effects (font (size {size} {size})){j}{h}))')


def _lib_pin(ptype, x, y, angle, length, name, number):
    return (f'        (pin {ptype} line (at {x} {y} {angle}) (length {length})\n'
            f'          (name "{name}" (effects (font (size 1.016 1.016))))\n'
            f'          (number "{number}" (effects (font (size 1.016 1.016)))))')


def _lib_symbol(name, w, h, ref_pfx, footprint, datasheet, pins_list):
    rx, ry = round(w / 2 + 1.5, 3), round(h / 2 + 1.5, 3)
    lines = [
        f'    (symbol "{name}"',
        f'      (pin_names (offset 1.016))',
        f'      (in_bom yes) (on_board yes)',
        _prop("Reference", ref_pfx, rx, ry),
        _prop("Value", name, rx, round(ry - 2.5, 3)),
        _prop("Footprint", footprint, 0, 0, hide=True),
        _prop("Datasheet", datasheet, 0, 0, hide=True),
        f'      (symbol "{name}_0_1"',
        f'        (rectangle (start {-w/2} {-h/2}) (end {w/2} {h/2})',
        f'          (stroke (width 0.254) (type default))',
        f'          (fill (type background)))',
        f'      )',
        f'      (symbol "{name}_1_1"',
    ]
    lines += pins_list
    lines += ['      )', '    )']
    return '\n'.join(lines)


def _sym_inst(lib_id, x, y, ref, value, footprint, uid_base, pin_count):
    lines = [
        f'  (symbol (lib_id "{lib_id}") (at {x} {y} 0) (unit 1)',
        f'    (in_bom yes) (on_board yes)',
        f'    (uuid "{u(uid_base)}")',
        _prop("Reference", ref, x + 3, y - 2, justify="left"),
        _prop("Value", value, x + 3, y - 4, justify="left"),
        _prop("Footprint", footprint, x, y, hide=True),
    ]
    for i in range(1, pin_count + 1):
        lines.append(f'    (pin "{i}" (uuid "{u(uid_base + i)}"))')
    lines.append('  )')
    return '\n'.join(lines)


def _wire(x1, y1, x2, y2, uid_n):
    return (f'  (wire (pts (xy {x1} {y1}) (xy {x2} {y2}))\n'
            f'    (stroke (width 0) (type default))\n'
            f'    (uuid "{u(uid_n)}"))')


def _label(text, x, y, angle, uid_n):
    just = "right" if angle in (180, 270) else "left"
    return (f'  (label "{text}" (at {x} {y} {angle})\n'
            f'    (fields_autoplaced yes)\n'
            f'    (effects (font (size 1.27 1.27)) (justify {just}))\n'
            f'    (uuid "{u(uid_n)}"))')


def _no_connect(x, y, uid_n):
    return f'  (no_connect (at {x} {y}) (uuid "{u(uid_n)}"))'


def _text(content, x, y, uid_n, size=1.5):
    return (f'  (text "{content}" (at {x} {y} 0)\n'
            f'    (effects (font (size {size} {size})) (justify left))\n'
            f'    (uuid "{u(uid_n)}"))')


def _schematic(title, date, rev, company, c1, c2,
               lib_syms, body_items, paper="A3"):
    parts = [
        '(kicad_sch',
        '  (version 20231120)',
        '  (generator "eeschema")',
        '  (generator_version "7.0.11")',
        f'  (paper "{paper}")',
        '  (title_block',
        f'    (title "{title}")',
        f'    (date "{date}")',
        f'    (rev "{rev}")',
        f'    (company "{company}")',
        f'    (comment 1 "{c1}")',
        f'    (comment 2 "{c2}")',
        '  )',
        '',
        '  (lib_symbols',
    ]
    parts += lib_syms
    parts += [
        '  )',
        '',
    ]
    parts += body_items
    parts += [
        '',
        '  (sheet_instances',
        '    (path "/" (page "1")))',
        ')',
    ]
    return '\n'.join(parts)


# ── Motor-Driver schematic ────────────────────────────────────────────────────

def build_motor_driver():
    # DRV8353RS lib_symbol  (19 logical pins, numbered 1-19)
    drv_pins = []
    left_pins = [
        ("power_in",  "VM",      1), ("power_in",  "VCP",    2),
        ("power_out", "DVDD",    3), ("power_in",  "GND",    4),
        ("input",     "ENABLE",  5), ("input",     "SCS",    6),
        ("input",     "SCLK",    7), ("input",     "SDI",    8),
        ("output",    "SDO",     9), ("output",    "nFAULT", 10),
    ]
    right_pins = [
        ("output", "GH_A", 11), ("output", "GL_A", 12),
        ("output", "GH_B", 13), ("output", "GL_B", 14),
        ("output", "GH_C", 15), ("output", "GL_C", 16),
        ("input",  "SH_A", 17), ("input",  "SH_B", 18),
        ("input",  "SH_C", 19),
    ]
    for i, (ptype, pname, pnum) in enumerate(left_pins):
        y = round(7.62 - i * 2.54, 3)
        drv_pins.append(_lib_pin(ptype, -15.24, y, 0, 5.08, pname, str(pnum)))
    for i, (ptype, pname, pnum) in enumerate(right_pins):
        y = round(7.62 - i * 2.54, 3)
        drv_pins.append(_lib_pin(ptype, 15.24, y, 180, 5.08, pname, str(pnum)))

    drv_sym = _lib_symbol(
        "DRV8353RS", 20, 22, "U",
        "Package_SO:VQFN-40_6x6mm_P0.5mm",
        "https://www.ti.com/lit/ds/symlink/drv8353.pdf",
        drv_pins,
    )

    # IRF2907 NMOS (G=1, D=2, S=3)
    mos_pins = [
        _lib_pin("input",   -5.08, 0,    0,   2.54, "G", "1"),
        _lib_pin("passive",  0,    3.81, 270, 2.54, "D", "2"),
        _lib_pin("passive",  0,   -3.81,  90, 2.54, "S", "3"),
    ]
    mos_sym = _lib_symbol(
        "IRF2907", 6, 8, "Q",
        "Package_TO_SOT_THT:TO-247-3_Vertical",
        "https://www.infineon.com/dgdl/irf2907pbf.pdf",
        mos_pins,
    )

    # 6-pin SPI connector (numbered 1-6)
    conn6_pins = [
        _lib_pin("passive", -5.08, y, 0, 2.54, f"Pin_{i}", str(i))
        for i, y in enumerate([6.35, 3.81, 1.27, -1.27, -3.81, -6.35], start=1)
    ]
    conn6_sym = _lib_symbol("Conn_6pin", 6, 14, "J",
                            "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical",
                            "~", conn6_pins)

    # 3-pin motor connector
    conn3_pins = [
        _lib_pin("passive", -5.08, y, 0, 2.54, f"Pin_{i}", str(i))
        for i, y in enumerate([2.54, 0, -2.54], start=1)
    ]
    conn3_sym = _lib_symbol("Conn_3pin", 6, 8, "J",
                            "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
                            "~", conn3_pins)

    # 2-pin power connector
    conn2_pins = [
        _lib_pin("passive", -5.08, 1.27, 0, 2.54, "Pin_1", "1"),
        _lib_pin("passive", -5.08, -1.27, 0, 2.54, "Pin_2", "2"),
    ]
    conn2_sym = _lib_symbol("Conn_2pin", 6, 6, "J",
                            "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                            "~", conn2_pins)

    lib_syms = [drv_sym, mos_sym, conn6_sym, conn3_sym, conn2_sym]

    body = []

    # U1 — DRV8353RS  (uid_base=1000, pins 1001-1019)
    body.append(_sym_inst("DRV8353RS", 100, 120, "U1", "DRV8353RS",
                          "Package_SO:VQFN-40_6x6mm_P0.5mm", 1000, 19))

    # Q1H..Q3H high-side MOSFETs (uid_base 2000/2100/2200)
    for idx, (ref, x, y, base) in enumerate([
        ("Q1H", 155, 95, 2000), ("Q2H", 155, 115, 2100), ("Q3H", 155, 135, 2200)
    ]):
        body.append(_sym_inst("IRF2907", x, y, ref, "IRF2907",
                              "Package_TO_SOT_THT:TO-247-3_Vertical", base, 3))

    # Q1L..Q3L low-side MOSFETs (uid_base 3000/3100/3200)
    for ref, x, y, base in [
        ("Q1L", 195, 95, 3000), ("Q2L", 195, 115, 3100), ("Q3L", 195, 135, 3200)
    ]:
        body.append(_sym_inst("IRF2907", x, y, ref, "IRF2907",
                              "Package_TO_SOT_THT:TO-247-3_Vertical", base, 3))

    # J1 — 48V power input
    body.append(_sym_inst("Conn_2pin", 55, 120, "J1", "48V_IN",
                          "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                          4000, 2))

    # J2 — Motor phase output
    body.append(_sym_inst("Conn_3pin", 55, 140, "J2", "MOTOR_PHASES",
                          "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
                          4100, 3))

    # J3 — SPI control from MCU
    body.append(_sym_inst("Conn_6pin", 55, 160, "J3", "SPI_CTRL",
                          "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical",
                          4200, 6))

    # Net labels
    uid = 6000
    for net, x, y, ang in [
        ("VM_BUS",  88, 128, 0),   ("VM_BUS",  151, 90, 0),
        ("GH_A",   113, 128, 180), ("GH_A",    151, 95, 0),
        ("GL_A",   113, 125, 180), ("GL_A",    191, 95, 0),
        ("GH_B",   113, 120, 180), ("GH_B",    151, 115, 0),
        ("GL_B",   113, 117, 180), ("GL_B",    191, 115, 0),
        ("GH_C",   113, 112, 180), ("GH_C",    151, 135, 0),
        ("GL_C",   113, 110, 180), ("GL_C",    191, 135, 0),
        ("PHASE_A", 165, 92, 0),   ("PHASE_B", 165, 112, 0),
        ("PHASE_C", 165, 132, 0),
        ("SPI_SCS",  113, 108, 180), ("SPI_SCLK", 113, 105, 180),
        ("SPI_SDI",  113, 102, 180), ("SPI_SDO",  113, 99,  180),
        ("DRV_ENABLE", 113, 96, 180), ("nFAULT",  113, 93, 180),
    ]:
        body.append(_label(net, x, y, ang, uid))
        uid += 1

    # Design note
    body.append(_text(
        "NOTE: One board per joint (x7 total). U2..U7 = identical DRV8353RS.",
        55, 80, uid, size=1.5))

    return _schematic(
        "Motor Driver - 7-DoF Arm Joint", "2026-05-03", "1.0",
        "Northeastern University",
        "DRV8353RS 3-phase gate driver + 6x IRF2907 half-bridge",
        "48V bus / 15A cont / 25A peak - one board per joint (x7)",
        lib_syms, body,
    )


# ── Power-Distribution schematic ─────────────────────────────────────────────

def build_power_distribution():
    # Fuse (2 pins)
    fuse_pins = [
        _lib_pin("passive", -5.08,  0, 0, 2.54, "A", "1"),
        _lib_pin("passive",  5.08,  0, 180, 2.54, "K", "2"),
    ]
    fuse_sym = _lib_symbol("Fuse_32A", 8, 4, "F",
                           "Fuse:Fuse_Littelfuse_NANO2_154Series_L6.1mm_W2.6mm_P10.9mm",
                           "~", fuse_pins)

    # TVS diode (2 pins)
    tvs_pins = [
        _lib_pin("passive", -5.08, 0, 0, 2.54, "A", "1"),
        _lib_pin("passive",  5.08, 0, 180, 2.54, "K", "2"),
    ]
    tvs_sym = _lib_symbol("D_TVS_P6KE58A", 8, 4, "D",
                          "Diode_THT:D_DO-15_P12.70mm_Horizontal",
                          "https://www.littelfuse.com/~/media/electronics/datasheets/tvs_diodes/littelfuse_tvs_diode_p6ke_datasheet.pdf.pdf",
                          tvs_pins)

    # IRF2907 E-stop FET (G=1, D=2, S=3)
    mos_pins = [
        _lib_pin("input",   -5.08, 0,    0,   2.54, "G", "1"),
        _lib_pin("passive",  0,    3.81, 270, 2.54, "D", "2"),
        _lib_pin("passive",  0,   -3.81,  90, 2.54, "S", "3"),
    ]
    mos_sym = _lib_symbol("IRF2907_ESTOP", 6, 8, "Q",
                          "Package_TO_SOT_THT:TO-247-3_Vertical",
                          "https://www.infineon.com/dgdl/irf2907pbf.pdf",
                          mos_pins)

    # LMR36520 buck regulator (8 pins)
    lmr_pins = [
        _lib_pin("power_in",  -10.16, 7.62,   0, 5.08, "VIN",  "1"),
        _lib_pin("power_in",  -10.16, 5.08,   0, 5.08, "AGND", "2"),
        _lib_pin("input",     -10.16, 2.54,   0, 5.08, "EN",   "3"),
        _lib_pin("input",     -10.16, 0,      0, 5.08, "FB",   "4"),
        _lib_pin("output",     10.16, 7.62, 180, 5.08, "SW",   "5"),
        _lib_pin("input",      10.16, 5.08, 180, 5.08, "BOOT", "6"),
        _lib_pin("power_out",  10.16, 2.54, 180, 5.08, "VOUT", "7"),
        _lib_pin("power_in",   10.16, 0,   180, 5.08, "PGND", "8"),
    ]
    lmr_sym = _lib_symbol("LMR36520", 16, 12, "U",
                          "Package_TO_SOT_SMD:SOT-23-8",
                          "https://www.ti.com/lit/ds/symlink/lmr36520.pdf",
                          lmr_pins)

    # TLV1117-33 LDO (3 pins)
    ldo_pins = [
        _lib_pin("power_in",  -5.08,  2.54, 0,   2.54, "IN",  "1"),
        _lib_pin("power_in",  -5.08, -2.54, 0,   2.54, "GND", "2"),
        _lib_pin("power_out",  5.08,  0,   180, 2.54, "OUT", "3"),
    ]
    ldo_sym = _lib_symbol("TLV1117-33", 8, 8, "U",
                          "Package_TO_SOT_THT:TO-252-2",
                          "https://www.ti.com/lit/ds/symlink/tlv1117.pdf",
                          ldo_pins)

    # 2-pin connectors
    conn2_pins = [
        _lib_pin("passive", -5.08,  1.27, 0, 2.54, "Pin_1", "1"),
        _lib_pin("passive", -5.08, -1.27, 0, 2.54, "Pin_2", "2"),
    ]
    conn2_sym = _lib_symbol("Conn_2pin", 6, 6, "J",
                            "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                            "~", conn2_pins)

    lib_syms = [fuse_sym, tvs_sym, mos_sym, lmr_sym, ldo_sym, conn2_sym]
    body = []

    # F1 — 32A fuse
    body.append(_sym_inst("Fuse_32A", 80, 80, "F1", "32A/250V",
                          "Fuse:Fuse_Littelfuse_NANO2_154Series_L6.1mm_W2.6mm_P10.9mm",
                          1000, 2))

    # D1 — P6KE58A TVS
    body.append(_sym_inst("D_TVS_P6KE58A", 100, 80, "D1", "P6KE58A",
                          "Diode_THT:D_DO-15_P12.70mm_Horizontal",
                          2000, 2))

    # Q1 — E-stop IRF2907
    body.append(_sym_inst("IRF2907_ESTOP", 120, 80, "Q1", "IRF2907",
                          "Package_TO_SOT_THT:TO-247-3_Vertical",
                          3000, 3))

    # U1 — LMR36520 48V→5V buck
    body.append(_sym_inst("LMR36520", 80, 110, "U1", "LMR36520",
                          "Package_TO_SOT_SMD:SOT-23-8",
                          4000, 8))

    # U2 — TLV1117-33 5V→3.3V LDO
    body.append(_sym_inst("TLV1117-33", 120, 110, "U2", "TLV1117-33",
                          "Package_TO_SOT_THT:TO-252-2",
                          5000, 3))

    # Connectors
    body.append(_sym_inst("Conn_2pin", 50, 80, "J1", "48V_BATT_IN (XT60)",
                          "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                          6000, 2))
    body.append(_sym_inst("Conn_2pin", 50, 95, "J2", "VM_OUT (48V to drivers)",
                          "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                          6100, 2))
    body.append(_sym_inst("Conn_2pin", 50, 110, "J3", "+5V_OUT",
                          "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                          6200, 2))
    body.append(_sym_inst("Conn_2pin", 50, 125, "J4", "+3V3_OUT",
                          "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
                          6300, 2))

    # Net labels
    uid = 8000
    for net, x, y, ang in [
        ("48V_RAW",    75,  80,  0), ("48V_FUSED", 95,  80,  0),
        ("48V_PROT",  115,  80,  0), ("VM_BUS",   130,  80,  0),
        ("ESTOP_GATE", 115,  85,  0),
        ("+5V",        75, 110,  0), ("+3V3",     115, 110,  0),
        ("SW_NODE",    75, 115, 180),
    ]:
        body.append(_label(net, x, y, ang, uid))
        uid += 1

    body.append(_text(
        "Power tree: 48V battery -> 32A fuse -> TVS clamp -> E-stop FET -> motor drivers",
        50, 65, uid, size=1.27))
    uid += 1
    body.append(_text(
        "LMR36520: 48V->5V/3A buck  |  TLV1117-33: 5V->3.3V/800mA LDO",
        50, 62, uid, size=1.27))

    return _schematic(
        "Power Distribution - 7-DoF Arm", "2026-05-03", "1.0",
        "Northeastern University",
        "48V / 32A fuse / P6KE58A TVS / IRF2907 E-stop / LMR36520 / TLV1117-33",
        "Power budget: ~293W peak across 7 joints",
        lib_syms, body,
    )


# ── Sensor-Interface schematic ────────────────────────────────────────────────

def build_sensor_interface():
    # AS5047P encoder (8 pins)
    as_pins = [
        _lib_pin("power_in",  -10.16,  5.08, 0,   5.08, "VDD3V3", "1"),
        _lib_pin("power_in",  -10.16,  2.54, 0,   5.08, "GND",    "2"),
        _lib_pin("input",     -10.16,  0,    0,   5.08, "CSn",    "3"),
        _lib_pin("input",     -10.16, -2.54, 0,   5.08, "CLK",    "4"),
        _lib_pin("input",      10.16,  5.08, 180, 5.08, "MOSI",   "5"),
        _lib_pin("output",     10.16,  2.54, 180, 5.08, "MISO",   "6"),
        _lib_pin("output",     10.16,  0,   180, 5.08, "PWM",    "7"),
        _lib_pin("output",     10.16, -2.54, 180, 5.08, "DAE",    "8"),
    ]
    as_sym = _lib_symbol("AS5047P", 16, 12, "U",
                         "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm",
                         "https://ams.com/documents/20143/36005/AS5047P_DS000324_3-00.pdf",
                         as_pins)

    # BNO055 IMU (10 pins)
    bno_pins = [
        _lib_pin("power_in",  -10.16,  8.89, 0,   5.08, "VDD",   "1"),
        _lib_pin("power_in",  -10.16,  6.35, 0,   5.08, "VDDIO", "2"),
        _lib_pin("power_in",  -10.16,  3.81, 0,   5.08, "GND",   "3"),
        _lib_pin("input",     -10.16,  1.27, 0,   5.08, "RESET", "4"),
        _lib_pin("input",     -10.16, -1.27, 0,   5.08, "PS0",   "5"),
        _lib_pin("input",     -10.16, -3.81, 0,   5.08, "PS1",   "6"),
        _lib_pin("bidirectional", 10.16, 8.89, 180, 5.08, "SDA", "7"),
        _lib_pin("input",      10.16,  6.35, 180, 5.08, "SCL",   "8"),
        _lib_pin("output",     10.16,  3.81, 180, 5.08, "INT",   "9"),
        _lib_pin("input",      10.16,  1.27, 180, 5.08, "ADR",  "10"),
    ]
    bno_sym = _lib_symbol("BNO055", 16, 16, "U",
                          "Package_LCC:LCC-28_7.0x7.0mm_P0.65mm",
                          "https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf",
                          bno_pins)

    # 5-pin SPI connector
    conn5_pins = [
        _lib_pin("passive", -5.08, y, 0, 2.54, f"Pin_{i}", str(i))
        for i, y in enumerate([5.08, 2.54, 0, -2.54, -5.08], start=1)
    ]
    conn5_sym = _lib_symbol("Conn_SPI_5pin", 6, 12, "J",
                            "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
                            "~", conn5_pins)

    # 4-pin I2C connector
    conn4_pins = [
        _lib_pin("passive", -5.08, y, 0, 2.54, f"Pin_{i}", str(i))
        for i, y in enumerate([3.81, 1.27, -1.27, -3.81], start=1)
    ]
    conn4_sym = _lib_symbol("Conn_I2C_4pin", 6, 10, "J",
                            "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
                            "~", conn4_pins)

    lib_syms = [as_sym, bno_sym, conn5_sym, conn4_sym]
    body = []

    # J1 — SPI connector from MCU
    body.append(_sym_inst("Conn_SPI_5pin", 50, 90, "J1", "SPI_MCU",
                          "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",
                          1000, 5))

    # U1 — AS5047P encoder (Joint 0)
    body.append(_sym_inst("AS5047P", 100, 90, "U1", "AS5047P",
                          "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm",
                          2000, 8))

    # U8 — BNO055 IMU (one shared IMU)
    body.append(_sym_inst("BNO055", 100, 130, "U8", "BNO055",
                          "Package_LCC:LCC-28_7.0x7.0mm_P0.65mm",
                          3000, 10))

    # J2 — I2C connector from MCU
    body.append(_sym_inst("Conn_I2C_4pin", 50, 130, "J2", "I2C_MCU",
                          "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
                          4000, 4))

    # Net labels
    uid = 6000
    for net, x, y, ang in [
        ("SPI_MOSI", 75, 88, 0),  ("SPI_MOSI", 85, 88, 0),
        ("SPI_MISO", 75, 90, 0),  ("SPI_MISO", 85, 90, 0),
        ("SPI_CLK",  75, 92, 0),  ("SPI_CLK",  85, 92, 0),
        ("ENC_CS0",  75, 94, 0),  ("ENC_CS0",  85, 94, 0),
        ("+3V3",     75, 96, 0),  ("+3V3",     85, 96, 0),
        ("I2C_SDA",  75, 128, 0), ("I2C_SDA",  85, 128, 0),
        ("I2C_SCL",  75, 130, 0), ("I2C_SCL",  85, 130, 0),
        ("IMU_INT",  115, 126, 0),
    ]:
        body.append(_label(net, x, y, ang, uid))
        uid += 1

    body.append(_text(
        "NOTE: U2..U7 are identical AS5047P encoders for Joints 1-6.",
        50, 75, uid, size=1.27))
    uid += 1
    body.append(_text(
        "Each encoder uses separate CS line: ENC_CS0..ENC_CS6.",
        50, 72, uid, size=1.27))

    return _schematic(
        "Sensor Interface - 7-DoF Arm", "2026-05-03", "1.0",
        "Northeastern University",
        "AS5047P 14-bit SPI encoder (x7 joints) + BNO055 I2C IMU",
        "SPI: 10 MHz / I2C: 400 kHz (Fast-mode)",
        lib_syms, body,
    )


# ── Write files ───────────────────────────────────────────────────────────────

def write(path, content):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(content)
    print(f"  wrote {os.path.relpath(path, BASE)}")


if __name__ == "__main__":
    print("Generating KiCad 7 schematics...")
    write(os.path.join(BASE, "motor-driver", "motor-driver.kicad_sch"),
          build_motor_driver())
    write(os.path.join(BASE, "power-distribution", "power-distribution.kicad_sch"),
          build_power_distribution())
    write(os.path.join(BASE, "sensor-interface", "sensor-interface.kicad_sch"),
          build_sensor_interface())
    print("Done.")
