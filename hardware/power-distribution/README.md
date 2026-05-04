# Power Distribution

Reference material for the 48 V bus distribution and protection stage.

Key components:
- Fuse: Littelfuse 0218032 (32 A, fast-blow ceramic)
- TVS: P6KE58A (58 V clamp, regen spike suppression)
- E-stop FET: IRF2907 (gate driven by MCU GPIO)
- Buck: LMR36520 (48 V → 5 V, 3 A)
- LDO: TLV1117-33 (5 V → 3.3 V, 800 mA)

See docs/architecture/power_tree.md for the full block diagram and budget.
Drop BOM CSVs, component specs, or wiring diagrams here.
