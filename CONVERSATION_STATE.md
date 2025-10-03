# Project Session Snapshot (2025-10-03)

## Purpose
Preserve current context before closing VS Code so development can resume smoothly after system update.

## Active Branch
`300baudFSK`

## Implemented Features
- Dedicated FSK transmitter (`src/FskTx.cpp`) with extended frame format and CRC-16 X.25.
- Dedicated FSK receiver (`src/FskRx.cpp`) performing frame parse, CRC validate, OLED display, LED blink.
- Extended payload fields: version tag (Vhhhh) and power source indicator (P?).
- Configurable beacon/status interval via `CONFIG_FSK_INTERVAL_MS` (TX=10000 ms, RX=8000 ms in `platformio.ini`).
- LED blink on TX and RX (pin 38 default, override with `-DSTATUS_LED_PIN=<pin>`).

## Frame Format
```
$BPF,<fc>,<upt_ms>,<vbat_mv>,<last_rssi>,<tempC>,Vhhhh,Ps*CCCC\n
```
- CRC: 16-bit X.25 over text between `$` and `*`, presented as 4 uppercase hex digits.
- Power source codes: B=battery present, E=external/no battery, ?=unknown.

## Key Files
- `platformio.ini` (FSK TX/RX envs with `build_src_filter` and version/interval flags)
- `src/FskTx.cpp` (transmitter logic)
- `src/FskRx.cpp` (receiver logic)
- `archive/Beacon_legacy.cpp` (historical combined logic)

## Outstanding Todo
- LoRa (CSS) init failure debugging (not yet re-prioritized) – see todo item #13.

## How To Build/Upload (after restart)
PlatformIO UI: select environment, click Build / Upload.
CLI (ensure `pio` in PATH):
```
pio run -e T_Beam_BPF_FSK_TX
pio run -e T_Beam_BPF_FSK_TX -t upload
pio run -e T_Beam_BPF_FSK_RX
pio run -e T_Beam_BPF_FSK_RX -t upload
```
Add `--upload-port COMx` if auto-detect fails on Windows.

## Quick Resumption Checklist
1. Pull latest (if working across machines): `git pull origin 300baudFSK`.
2. Build TX and RX envs to re-hydrate `.pio` directories.
3. Open Serial Monitor at 115200 baud.
4. Flash TX unit, confirm immediate first frame.
5. Flash RX unit, verify packet count increments and fields decode.

## Next Potential Enhancements
- Implement real temperature sensor or PMU internal temperature if available.
- Improve RSSI reporting (current payload embeds last known value; could sample channel RSSI before/after TX).
- Add NVS persistence of frame counter.
- Add power metrics (battery percent, charge current) when PMU supports it.
- Resolve legacy LoRa init issue and optionally maintain dual-mode build.

----
Snapshot generated on 2025-10-03.
