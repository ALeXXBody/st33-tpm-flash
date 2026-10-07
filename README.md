# ST33 TPM Flasher

Native Windows GUI + CLI tool to **directly flash and repair the STMicroelectronics
ST33HTPH2X32 (ST33TPHF2XSPI family) TPM 2.0 chip** over SPI with a CH341A USB adapter —
built to fix the **"TPM device is not detected"** failure that otherwise bricks the
discrete TPM on these boards.

Built for the **Dell XPS 15 9510** (TPM at U32, SPI chip select `CS#2`), but the
protocol applies to every ST33TPHF2XSPI-family part (ST33HTPH2X32AHD4/D8/E4, ...).

## The problem it solves

On affected units the discrete TPM *is on the board and powered*, the SPI lines are
sound, and the BIOS still refuses to enumerate it: instead it presents Intel PTT
("only PTT"), Windows sees no TPM, and Dell's own updater (`DELLTPM_STHE4_*.exe`)
refuses to run because it only drives a TPM the OS can already see.

Decoded from the board's own firmware, the BIOS driver carries explicit handling for
this exact failure: `TIS_ERR_TpmNotReady`, `TPM is in FU mode`,
`Blocked by TPM FW policy` — the chip is stuck mid field-upgrade and never presents
its TIS interface again, so **no host-side updater can reach it**.

The fix is to reach the chip directly, on the bench, and stream a valid signed
firmware payload to it.

## How it works

The tool speaks the **TCG PTP 2.0 FIFO/TIS register protocol over SPI**, byte-exact with
the framing the Linux kernel `tpm_tis_spi` driver uses:

```
READ   { 0x80 | (n-1), 0xD4, addr>>8, addr&0xFF }  wait bytes (bit0=0), 'ready' token, n data bytes
WRITE  { 0x00 | (n-1), 0xD4, addr>>8, addr&0xFF }  n data bytes
```

* registers: `ACCESS 0x0000, INT_ENABLE 0x0008, INT_STATUS 0x000C, INTF_CAP 0x0010,
  STS 0x0018, DATA_FIFO 0x0024, DID_VID 0x0F00, RID 0x0F04` (little-endian words,
  ≤ 64 bytes per CS assertion, FIFO auto-advances)
* TPM2 commands used:
  * `TPM2_GetCapability(TPM_PT_FIRMWARE_VERSION_1)` — probe whether the TPM2 layer lives
  * `TPM2_FieldUpgradeData` — streams the signed payload into the loader
* transport through the vendor `CH341DLL.dll` (CH341A)

A **built-in simulator** reproduces the chip behaviour for offline rehearsal:

| mode      | behaviour |
|-----------|-----------|
| `healthy` | full TIS + command handling; FU streaming completes |
| `fu_stuck`| registers answer, every TPM2 command is refused (`TPM_RC 0x00000101`) — mirrors the real broken state |
| `bus_dead`| chip absent; reads float (the tool reports "no chip response" and refuses to flash) |

## Bench wiring (Dell XPS 15 9510, LA-K411P)

Isolate the PCH so only the CH341A drives the chip: cut/desolder **RE133**
(PCH-side of `PCH_SPI_CS#2`) or lift **U32 pin 20**.

| CH341A | board |
|--------|-------|
| `CS0`  | U32 pin 20 (chip side of the cut) |
| `MOSI` | U32 pin 21 (`SPI_SI`) |
| `MISO` | U32 pin 24 (`SPI_SO`) |
| `SCK`  | U32 pin 19 (`SPI_CLK`) |
| `GND`  | board ground |
| *(power)* | inject **3.3 V** onto `+3.3V_VPS_TPM` (U32 pin 22, convenient at the CE11 pad). Laptop unplugged, battery removed. Do not tie the CH341A's own 3.3 V onto that rail. |

## Using it

### GUI (default)
Double-click `st33tpmtool.exe`:

1. tick *Demo: run simulator instead* to rehearse the full flow with no hardware, or
2. untick it, **Open** the CH341A, **Probe** — expect `VID=0x104A (STMicroelectronics)`,
3. **Caps** — a healthy chip answers with its firmware version; a stuck one refuses
   with `TPM_RC = 0x00000101`,
4. load the payload, dry-run first, then **FLASH FIRMWARE** (one-way!).

### CLI
```bat
st33tpmtool.exe --cli info                       identify chip (VID/DID/RID)
st33tpmtool.exe --cli status                     ACCESS / STS / INTF_CAP registers
st33tpmtool.exe --cli caps                       TPM2_GetCapability (firmware version)
st33tpmtool.exe --cli fu --file fw.bin --dry     dry-run
st33tpmtool.exe --cli fu --file fw.bin           the real flash
st33tpmtool.exe --cli --sim fu_stuck test        run the self-test offline
```

`--sim healthy|fu_stuck|bus_dead` forces the built-in simulator for any verb.

Everything gets written to `st33tpmtool.log` next to the .exe.

## The firmware payload

**Deliberately not included in this repository** (it is Dell/ST signed property).
Extract your own from the official Dell tool you already have for your service tag,
in three steps:

1. `7z x DELLTPM_STHE4_1.771_64.exe` — gives you section blobs (`[0]`, `.data`, …)
2. inside the blob, at the zlib markers, decompress twice:
   raw bytes → `PFS.HDR.` container → zlib stream →
3. the second zlib payload (`PFS.HDR.` rev 2) unwraps to
   `TPM 2.0 v1.771.bin` — that is the signed field-upgrade image.

## Build

Cross-compile on Linux:
```sh
x86_64-w64-mingw32-gcc -O2 -mwindows -o st33tpmtool.exe src/st33tpmtool.c
```
Or let the CI do it (`.github/workflows/build.yml` builds the .exe and runs the
integration test-suite under Wine on every push).

## Tests

`tests/test_st33tpmtool.py` drives the **compiled .exe** (under Wine on CI, or natively
on Windows) against the simulator — self-test, registration, refusals, dry-run,
silent-bus behaviour. Runs green; 9 cases. The executable path is taken from
`ST33TPMTOOL_EXE` if set.

```sh
python -m venv .venv && . .venv/bin/activate && pip install pytest
pytest tests -q
```

## Notes and warnings

* the ST33 has a **limited firmware-update count** — flashing is one-way; rehearse
  in simulator mode first,
* board-repair tool for qualified technicians: hot air, soldering skills and
  proper ESD practice required; not affiliated with Dell, ST or TCG,
* the TCG PTP register protocol details were cross-checked against the
  publicly documented Linux driver behaviour and ST's published datasheet
  (DB3421/DS14186).

## License

MIT — see [LICENSE](LICENSE).
