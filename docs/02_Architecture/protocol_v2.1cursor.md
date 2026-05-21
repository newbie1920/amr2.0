# Protocol — Firmware 2.1cursor

Compatible with existing dashboard `robotProtocol.js`. New telemetry fields:

| Field | Type | Description |
|-------|------|-------------|
| `firmware` | string | `"2.1cursor"` |
| `schema` | int | `21` |

Commands unchanged (`goto`, `navigate`, `reset_odom`, `set_mode`, `set_arch_mode`, …).

Manual drive uses firmware `MotionService` + `DiffDrive` (same wire format: `linear` + `angular`).
