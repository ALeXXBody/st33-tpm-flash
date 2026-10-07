"""Integration tests for st33tpmtool.exe — drive the compiled Windows binary
under Wine, with the built-in simulator standing in for the chip.

Run with the `.venv` interpreter:
    /root/venvs/st33/bin/pytest -q tests
"""
import subprocess
import pytest

EXE = "/home/truenas_admin/drop/st33tpmtool.exe"
FW  = "/home/truenas_admin/drop/public/st33_v1771.bin"

ENV = {
    "PATH": "/usr/bin:/bin:/usr/local/bin",
    "DISPLAY": "",
    "WINEDEBUG": "-all",
    "WINEPREFIX": "/root/.wine64",
    "XDG_RUNTIME_DIR": "/run",
}


def run(*args, exe=EXE, env=ENV):
    cmd = ["wine", exe, "--cli", *args]
    p = subprocess.run(cmd, capture_output=True, text=True, env=env, timeout=120)
    return p.returncode, p.stdout + p.stderr


def test_selftest_passes():
    rc, out = run("--sim", "test")
    assert "=== healthy ===" in out
    assert "=== fu_stuck ===" in out
    assert "=== bus_dead ===" in out
    assert "SELFTEST PASS" in out
    assert rc == 0, out


def test_info_healthy():
    rc, out = run("--sim", "healthy", "info")
    assert rc == 0
    assert "VID=0x104A" in out
    assert "STMicroelectronics" in out
    assert "DID=0x0001" in out


def test_info_fustuck_regs_still_answer():
    """Frozen chips still expose DID_VID — that's what we saw on the real chip."""
    rc, out = run("--sim", "fu_stuck", "info")
    assert rc == 0
    assert "VID=0x104A" in out


def test_caps_fustuck_refused():
    rc, out = run("--sim", "fu_stuck", "caps")
    assert rc != 0                      # refusal must be a non-zero exit
    assert "TPM_RC = 0x00000101" in out # refusal reason made it out to the caller


def test_caps_firmware_version_healthy():
    rc, out = run("--sim", "healthy", "caps")
    assert rc == 0
    assert "TPM_RC = 0x00000000" in out
    assert "TPM_PT_FIRMWARE_VERSION_1 = 0x01077100" in out


def test_bus_dead_clean_failure():
    """No chip: tool must report 'no chip response' and exit cleanly, not crash."""
    rc, out = run("--sim", "bus_dead", "info")
    assert "no chip response" in out
    assert "VID=0x0000" not in out   # must NOT fabricate a VID from floating bus
    assert rc == 0


def test_status_fields_present():
    rc, out = run("--sim", "healthy", "status")
    assert rc == 0
    assert "TPM_STS = 0x" in out
    assert "burst=" in out
    assert "dataAvail=" in out


def test_fu_dry_run_does_not_transmit():
    """Dry-run: tool must validate the payload exist and refuse to transmit."""
    rc, out = run("--sim", "healthy", "fu", "--file", FW, "--dry")
    assert rc == 0
    assert "--dry: not transmitting." in out


def test_unknown_verb_is_rejected():
    rc, out = run("--sim", "nonsense")
    assert rc != 0
    assert "unknown command" in out


def test_fu_drives_full_stream_against_sim():
    """Field-update path end-to-end against the healthy simulator."""
    rc, out = run("--sim", "healthy", "fu", "--file", FW)
    assert rc == 0, out
    assert "preflight: ST33 confirmed" in out
    assert "stream complete" in out


def test_fu_preflight_refused_on_dead_bus():
    """No chip identified -> fu must refuse before transmitting."""
    rc, out = run("--sim", "bus_dead", "fu", "--file", FW)
    assert rc != 0
    assert "refusing: no chip identified" in out


def test_driver_verb_supported():
    rc, out = run("--sim", "driver")
    assert "USB devices from WCH" in out
