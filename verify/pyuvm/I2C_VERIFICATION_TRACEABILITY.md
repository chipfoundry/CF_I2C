# CF_I2C — Verification traceability

## Scoreboard

- **IP path**: Lockstep I2C transaction compare; `check_phase` asserts `failed == 0`.

## Register smoke

- **WriteReadRegsTest**: `write_read_regs_seq` plus explicit readback for readable writable registers. `IM` is skipped: the mask register does not read back arbitrary data (reserved bits are cleared).
