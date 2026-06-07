"""Unit and mock-integration tests for arduino_client.ArduinoClient."""

from __future__ import annotations

import pytest

import arduino_client as ac


# ---------- Parser unit tests ----------


def test_parse_ack_line_well_formed():
    assert ac.parse_ack_line("ACK 12 HOME_ALL") == (12, "HOME_ALL")


def test_parse_ack_line_with_trailing_whitespace():
    assert ac.parse_ack_line("ACK 7 MOVE_X_ABS\r\n") == (7, "MOVE_X_ABS")


def test_parse_ack_returns_none_for_non_ack_line():
    assert ac.parse_ack_line("DONE 1 HOME_ALL x_mm=0.0") is None
    assert ac.parse_ack_line("garbage line") is None


def test_parse_done_line_typed_fields():
    line = (
        "DONE 101 GET_STATUS homed=true busy=false faulted=false "
        "x_mm=42.5 z_mm=12.5 rot_deg=90.0 blade_on=false vise_state=CLOSED "
        'force_kg=4.1 x_limit=false z_limit=false active_command=NONE'
    )
    cmd_id, command, fields = ac.parse_done_line(line)
    assert cmd_id == 101
    assert command == "GET_STATUS"
    assert fields["homed"] is True
    assert fields["busy"] is False
    assert fields["x_mm"] == 42.5
    assert fields["rot_deg"] == 90.0
    assert fields["vise_state"] == "CLOSED"
    assert fields["force_kg"] == 4.1
    assert fields["active_command"] == "NONE"


def test_parse_err_line_strips_message_quotes():
    line = 'ERR 99 MOVE_X_ABS code=NOT_HOMED message="Machine must be homed first"'
    cmd_id, command, code, message = ac.parse_err_line(line)
    assert cmd_id == 99
    assert command == "MOVE_X_ABS"
    assert code == "NOT_HOMED"
    assert message == "Machine must be homed first"


# ---------- Command formatter unit tests ----------


def test_format_command_floats_six_decimal_places():
    wire = ac.format_command("MOVE_X_ABS", 5, {"x_mm": 42.0, "feed": 25.0})
    assert wire == "MOVE_X_ABS cmd_id=5 x_mm=42.000000 feed=25.000000\n"


def test_format_command_omits_none_values():
    wire = ac.format_command("MOVE_X_ABS", 6, {"x_mm": 1.5, "feed": None})
    assert wire == "MOVE_X_ABS cmd_id=6 x_mm=1.500000\n"


def test_format_command_renders_booleans_as_on_off():
    wire = ac.format_command("SET_BLADE", 7, {"state": True})
    assert wire == "SET_BLADE cmd_id=7 state=ON\n"
    wire_off = ac.format_command("SET_BLADE", 8, {"state": False})
    assert wire_off == "SET_BLADE cmd_id=8 state=OFF\n"


def test_format_command_query_no_args():
    wire = ac.format_command("GET_STATUS", 12, None)
    assert wire == "GET_STATUS cmd_id=12\n"


def test_format_command_passes_strings_through():
    wire = ac.format_command("MOVE_REL", 13, {"axis": "X", "delta": -5.0})
    assert wire == "MOVE_REL cmd_id=13 axis=X delta=-5.000000\n"


# ---------- Mock-serial integration tests ----------


def _ack_done(fake, line_in: str, cmd: str, done_kv: str) -> None:
    """Auto-responder helper: emit ACK then DONE for the given input line."""
    # Parse the cmd_id from the wire ("CMD cmd_id=N ...").
    parts = line_in.split()
    cmd_id = None
    for part in parts:
        if part.startswith("cmd_id="):
            cmd_id = int(part.split("=", 1)[1])
            break
    assert cmd_id is not None, f"No cmd_id in {line_in!r}"
    fake.feed_response(f"ACK {cmd_id} {cmd}")
    fake.feed_response(f"DONE {cmd_id} {cmd} {done_kv}".rstrip())


def test_cmd_id_increments_per_instance(fake_serial_factory):
    captured = []

    def responder(fake, line):
        captured.append(line)
        _ack_done(fake, line, "HOME_ALL", "x_mm=0.0 z_mm=0.0 rot_deg=0.0 busy=false homed=true faulted=false")

    fake = fake_serial_factory(auto_responder=responder)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()

    r1 = client.home_all()
    r2 = client.home_all()
    r3 = client.home_all()
    assert (r1["cmd_id"], r2["cmd_id"], r3["cmd_id"]) == (1, 2, 3)
    # And starts at 1.
    assert captured[0].startswith("HOME_ALL cmd_id=1")


def test_home_all_roundtrip_parses_done_fields(fake_serial_factory):
    def responder(fake, line):
        _ack_done(
            fake, line, "HOME_ALL",
            "x_mm=0.0 z_mm=0.0 rot_deg=0.0 busy=false homed=true faulted=false",
        )

    fake = fake_serial_factory(auto_responder=responder)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()

    result = client.home_all()
    assert result["command"] == "HOME_ALL"
    assert result["cmd_id"] == 1
    assert result["homed"] is True
    assert result["busy"] is False
    assert result["x_mm"] == 0.0


def test_cut_height_serializes_required_args(fake_serial_factory):
    captured_lines = []

    def responder(fake, line):
        captured_lines.append(line)
        _ack_done(
            fake, line, "CUT_HEIGHT",
            "x_mm=0.0 z_mm=0.0 rot_deg=0.0 blade_on=false busy=false homed=true faulted=false",
        )

    fake = fake_serial_factory(auto_responder=responder, read_timeout=0.05)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()

    result = client.cut_height(z_mm=134.0, x_mm=111.0, deg=359.0)
    assert result["blade_on"] is False
    assert captured_lines == [
        "CUT_HEIGHT cmd_id=1 z_mm=134.000000 x_mm=111.000000 deg=359.000000",
    ]


def test_err_response_raises_arduino_command_error(fake_serial_factory):
    def responder(fake, line):
        parts = line.split()
        cmd_id = next(int(p.split("=", 1)[1]) for p in parts if p.startswith("cmd_id="))
        fake.feed_response(f"ACK {cmd_id} MOVE_X_ABS")
        fake.feed_response(
            f'ERR {cmd_id} MOVE_X_ABS code=NOT_HOMED message="Machine must be homed first"'
        )

    fake = fake_serial_factory(auto_responder=responder)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()

    with pytest.raises(ac.ArduinoCommandError) as excinfo:
        client.move_x_abs(42.0)
    err = excinfo.value
    assert err.code == "NOT_HOMED"
    assert err.message == "Machine must be homed first"
    assert err.command == "MOVE_X_ABS"
    assert err.cmd_id == 1


def test_timeout_raises_and_emits_stop_all(fake_serial_factory):
    captured = []

    def responder(fake, line):
        captured.append(line)
        # Deliberately emit no response at all -> client should time out.

    fake = fake_serial_factory(auto_responder=responder, read_timeout=0.02)
    client = ac.ArduinoClient(
        serial_factory=lambda: fake, done_timeout_s=0.1
    )
    client.connect()

    with pytest.raises(ac.ArduinoTimeoutError):
        client.home_all()

    # After timeout the client must have sent STOP_ALL on the wire (best-effort).
    assert any(line.startswith("STOP_ALL ") for line in captured), (
        f"expected STOP_ALL after timeout, saw: {captured}"
    )


def test_stray_lines_before_done_are_tolerated(fake_serial_factory):
    def responder(fake, line):
        parts = line.split()
        cmd_id = next(int(p.split("=", 1)[1]) for p in parts if p.startswith("cmd_id="))
        # Stray boot banner / log line before the structured response.
        fake.feed_response("[boot] cutting machine v2 ready")
        fake.feed_response(f"ACK {cmd_id} HOME_ALL")
        fake.feed_response("[info] homing rotary axis")
        fake.feed_response(
            f"DONE {cmd_id} HOME_ALL x_mm=0.0 z_mm=0.0 rot_deg=0.0 "
            f"busy=false homed=true faulted=false"
        )

    fake = fake_serial_factory(auto_responder=responder)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()

    result = client.home_all()
    assert result["homed"] is True
    stray = client.drain_stray_lines()
    assert any("boot" in line for line in stray)
    assert any("homing rotary" in line for line in stray)


def test_get_status_query_no_ack_required(fake_serial_factory):
    def responder(fake, line):
        parts = line.split()
        cmd_id = next(int(p.split("=", 1)[1]) for p in parts if p.startswith("cmd_id="))
        # Queries emit DONE directly per PRIMITIVES.md, no ACK.
        fake.feed_response(
            f"DONE {cmd_id} GET_STATUS homed=true busy=false faulted=false "
            f"x_mm=42.0 z_mm=12.5 rot_deg=90.0 blade_on=false "
            f"vise_state=CLOSED force_kg=4.1 x_limit=false z_limit=false "
            f"active_command=NONE"
        )

    fake = fake_serial_factory(auto_responder=responder)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()

    result = client.get_status()
    assert result["vise_state"] == "CLOSED"
    assert result["force_kg"] == 4.1
    assert result["x_mm"] == 42.0


def test_close_vise_target_force_kg_is_serialized(fake_serial_factory):
    captured = []

    def responder(fake, line):
        captured.append(line)
        _ack_done(
            fake, line, "CLOSE_VISE",
            "vise_state=CLOSED force_kg=4.1 busy=false homed=true faulted=false",
        )

    fake = fake_serial_factory(auto_responder=responder, read_timeout=0.05)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()

    result = client.close_vise(target_force_kg=4.0)
    assert result["vise_state"] == "CLOSED"
    assert captured == [
        "CLOSE_VISE cmd_id=1 target_force_kg=4.000000",
    ]


def test_move_rel_validates_axis(fake_serial_factory):
    fake = fake_serial_factory(auto_responder=lambda *_: None)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()
    with pytest.raises(ValueError):
        client.move_rel("Y", 1.0)


def test_set_blade_serializes_state_word(fake_serial_factory):
    captured = []

    def responder(fake, line):
        captured.append(line)
        _ack_done(
            fake, line, "SET_BLADE",
            "blade_on=true busy=false homed=true faulted=false",
        )

    fake = fake_serial_factory(auto_responder=responder)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()
    client.set_blade(True)
    assert captured[0] == "SET_BLADE cmd_id=1 state=ON"


def test_format_value_int_passthrough():
    wire = ac.format_command("HOME_ALL", 10, {})
    assert wire == "HOME_ALL cmd_id=10\n"


# ---------- READY banner handshake ----------


def test_connect_default_does_not_wait_for_ready(fake_serial_factory):
    # Default behavior (used by every existing test) must not block on a banner
    # that the fake serial never emits.
    fake = fake_serial_factory(auto_responder=None)
    client = ac.ArduinoClient(serial_factory=lambda: fake)
    client.connect()  # would hang if it waited for READY


def test_connect_waits_for_ready_banner(fake_serial_factory):
    fake = fake_serial_factory(auto_responder=None)
    # Pre-queue the boot banner so connect() can consume it.
    fake.feed_response("READY protocol=primitive_api as5600=true loadcell=true")
    fake.feed_response(
        "DONE 1 GET_STATUS homed=false busy=false faulted=false "
        "x_mm=0.0 z_mm=0.0 rot_deg=0.0 blade_on=false vise_state=OPEN "
        "force_kg=0.0 x_limit=false z_limit=false active_command=NONE"
    )

    def responder(_fake, _line):  # GET_STATUS issuance happens after connect
        pass  # responses pre-queued

    fake._auto_responder = responder
    client = ac.ArduinoClient(serial_factory=lambda: fake, ready_timeout_s=1.0)
    client.connect()

    result = client.get_status()
    assert result["vise_state"] == "OPEN"
    # The READY banner should not have been routed to the cmd response queue.
    assert all(not s.startswith("READY") for s in client.drain_stray_lines())


def test_connect_ready_timeout_does_not_raise(fake_serial_factory):
    # If the firmware was already running (no banner forthcoming), connect should
    # log a warning via the logger and return normally rather than raise.
    fake = fake_serial_factory(auto_responder=None, read_timeout=0.01)

    class _CaptureLogger:
        def __init__(self) -> None:
            self.infos: list[str] = []
            self.debugs: list[str] = []

        def info(self, msg):
            self.infos.append(msg)

        def debug(self, msg):
            self.debugs.append(msg)

    log = _CaptureLogger()
    client = ac.ArduinoClient(
        serial_factory=lambda: fake, ready_timeout_s=0.05, logger=log
    )
    client.connect()
    assert any("READY banner not seen" in m for m in log.infos)


# ---------- Error code constants ----------


def test_arduino_command_error_carries_metadata():
    err = ac.ArduinoCommandError(
        code="LIMIT_HIT", message="Travel limit reached", cmd_id=42, command="MOVE_X_ABS"
    )
    assert err.code == "LIMIT_HIT"
    assert err.cmd_id == 42
    assert err.command == "MOVE_X_ABS"
    assert "LIMIT_HIT" in str(err)
