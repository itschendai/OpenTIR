"""Flask backend for the cutting-cell GUI.

Run:  python GUI/app.py   (from the project dir, with the project venv active)

One process owns all hardware (see hardware.HardwareHub). Status is polled by the
frontend; the camera is an MJPEG stream; control endpoints enqueue operations on
the single shared executor.
"""

from __future__ import annotations

import atexit
import sys
import threading
import webbrowser
from pathlib import Path

# Make the GUI package importable when launched as `python GUI/app.py`.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from flask import Flask, Response, jsonify, render_template, request  # noqa: E402

import config  # noqa: E402
from hardware import HardwareHub  # noqa: E402

app = Flask(__name__)
hub = HardwareHub()


def _accepted(ok: bool):
    """Standard reply for executor-backed (one-at-a-time) operations."""
    if ok:
        return jsonify({"accepted": True}), 202
    return jsonify({"accepted": False, "reason": "busy",
                    "operation": hub.executor.snapshot()["operation"]}), 409


# ----- pages ----------------------------------------------------------------

@app.route("/")
def index():
    return render_template("index.html")


# ----- status + camera ------------------------------------------------------

@app.route("/api/status")
def api_status():
    return jsonify({
        "executor": hub.executor.snapshot(),
        "robot": hub.robot.status(),
        "machine": hub.machine.status(),
        "recipe": hub.recipes.state(),
    })


@app.route("/api/camera/stream")
def api_camera_stream():
    return Response(
        hub.camera.mjpeg_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ----- robot ----------------------------------------------------------------

@app.route("/api/waypoints")
def api_waypoints():
    return jsonify({"waypoints": hub.robot.list_waypoints()})


@app.route("/api/robot/home", methods=["POST"])
def api_robot_home():
    return _accepted(hub.robot.home())


@app.route("/api/robot/move_to", methods=["POST"])
def api_robot_move_to():
    name = (request.json or {}).get("name")
    if not name:
        return jsonify({"error": "name required"}), 400
    return _accepted(hub.robot.move_to(name))


@app.route("/api/robot/jog", methods=["POST"])
def api_robot_jog():
    body = request.json or {}
    return _accepted(hub.robot.jog(body["axis"], float(body["delta"])))


@app.route("/api/robot/gripper", methods=["POST"])
def api_robot_gripper():
    action = (request.json or {}).get("action", "open")
    return _accepted(hub.robot.gripper_set(action))


@app.route("/api/robot/floating", methods=["POST"])
def api_robot_floating():
    body = request.json or {}
    mode = body.get("mode", "cartesian")
    on = bool(body.get("on", True))
    selection = body.get("selection") or []
    return _accepted(hub.robot.floating(mode, on, selection))


@app.route("/api/robot/zero_ft", methods=["POST"])
def api_robot_zero_ft():
    return _accepted(hub.robot.zero_ft())


@app.route("/api/robot/clear_fault", methods=["POST"])
def api_robot_clear_fault():
    return jsonify(hub.robot.clear_fault())


@app.route("/api/robot/stop", methods=["POST"])
def api_robot_stop():
    return jsonify(hub.robot.stop())


# ----- machine + cutter -----------------------------------------------------

@app.route("/api/machine/home", methods=["POST"])
def api_machine_home():
    return _accepted(hub.machine.home())


@app.route("/api/machine/move_x", methods=["POST"])
def api_machine_move_x():
    body = request.json or {}
    return _accepted(hub.machine.move_x(float(body["x_mm"]), body.get("feed")))


@app.route("/api/machine/move_z", methods=["POST"])
def api_machine_move_z():
    body = request.json or {}
    return _accepted(hub.machine.move_z(float(body["z_mm"]), body.get("feed")))


@app.route("/api/machine/rotate", methods=["POST"])
def api_machine_rotate():
    body = request.json or {}
    return _accepted(hub.machine.rotate(float(body["deg"]), body.get("speed")))


@app.route("/api/machine/vise", methods=["POST"])
def api_machine_vise():
    body = request.json or {}
    return _accepted(hub.machine.vise(body.get("action", "open"), body.get("force_kg")))


@app.route("/api/machine/blade", methods=["POST"])
def api_machine_blade():
    body = request.json or {}
    return _accepted(hub.machine.blade(bool(body.get("on", False))))


@app.route("/api/machine/cut", methods=["POST"])
def api_machine_cut():
    body = request.json or {}
    return _accepted(hub.machine.cut(
        float(body["z_mm"]), float(body["x_mm"]), float(body["deg"])))


@app.route("/api/machine/clear", methods=["POST"])
def api_machine_clear():
    return jsonify(hub.machine.clear_faults())


@app.route("/api/machine/stop", methods=["POST"])
def api_machine_stop():
    return jsonify(hub.machine.stop())


# ----- recipes --------------------------------------------------------------

@app.route("/api/recipes")
def api_recipes():
    return jsonify({"recipes": hub.recipes.list_recipes()})


@app.route("/api/recipes/<name>")
def api_recipe_detail(name):
    try:
        return jsonify(hub.recipes.select(name))
    except Exception as exc:  # noqa: BLE001
        return jsonify({"error": str(exc)}), 400


@app.route("/api/recipe/run", methods=["POST"])
def api_recipe_run():
    body = request.json or {}
    return _accepted(hub.recipes.run_full(
        speed_scale=float(body.get("speed", 1.0)),
        loops=int(body.get("loops", 1)),
    ))


@app.route("/api/recipe/step", methods=["POST"])
def api_recipe_step():
    body = request.json or {}
    return _accepted(hub.recipes.step(
        phase_name=body.get("phase"),
        speed_scale=float(body.get("speed", 1.0)),
    ))


@app.route("/api/recipe/stop", methods=["POST"])
def api_recipe_stop():
    return jsonify(hub.recipes.stop())


@app.route("/api/recipe/log")
def api_recipe_log():
    after = int(request.args.get("after", 0))
    return jsonify({"records": hub.logger.records(after_seq=after)})


def _open_browser() -> None:
    # localhost regardless of bind address (HOST may be 0.0.0.0).
    url = f"http://localhost:{config.PORT}"
    try:
        webbrowser.open_new(url)
    except Exception:  # noqa: BLE001 - headless / no browser is fine
        print(f"Open the GUI at {url}", flush=True)


def main() -> int:
    hub.start()
    atexit.register(hub.close)
    # Pop the GUI once the server is about to accept connections. Short delay so
    # the browser hits a live socket; runs in a thread since app.run() blocks.
    threading.Timer(1.0, _open_browser).start()
    # threaded=True so the MJPEG stream and status polls are served concurrently.
    # use_reloader=False so we don't double-init the hardware (or open twice).
    app.run(host=config.HOST, port=config.PORT, threaded=True, use_reloader=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
