"""Generate state_machine_controller_slides.pdf (mirrors the .pptx deck).

Run: ../.venv/bin/python build_state_machine_pdf.py
"""
from reportlab.lib.colors import Color
from reportlab.pdfgen import canvas

# 16:9 page in points (13.333 x 7.5 in)
PW, PH = 13.333 * 72, 7.5 * 72


def C(h):
    return Color(int(h[0:2], 16) / 255, int(h[2:4], 16) / 255, int(h[4:6], 16) / 255)


DARK = C("1F2A44"); GREY = C("5B6372"); LIGHT = C("F2F4F8"); WHITE = C("FFFFFF")
BLUE = C("2E6FD6"); GREEN = C("2EA05A"); RED = C("D63B3B"); ACCENT = C("6B4EC4")
LBLUE = C("9DC4FF"); LRED = C("FFB3B3"); PANEL = C("2A3757"); SUBT = C("B9C6E6")
TXT2 = C("D9E2F2")

c = canvas.Canvas("/home/src0/flexiv_rdk/project/state_machine_controller_slides.pdf",
                  pagesize=(PW, PH))

# y helper: reportlab origin bottom-left; we author top-down in inches
def Y(top_in):
    return PH - top_in * 72


def IN(v):
    return v * 72


def rrect(x, y_top, w, h, fill=None, stroke=None, sw=1, r=8):
    c.saveState()
    if fill:
        c.setFillColor(fill)
    if stroke:
        c.setStrokeColor(stroke); c.setLineWidth(sw)
    c.roundRect(IN(x), Y(y_top + h), IN(w), IN(h), r,
                fill=1 if fill else 0, stroke=1 if stroke else 0)
    c.restoreState()


def rect(x, y_top, w, h, fill):
    c.saveState(); c.setFillColor(fill)
    c.rect(IN(x), Y(y_top + h), IN(w), IN(h), fill=1, stroke=0)
    c.restoreState()


def text(x, y_top, s, size, color, bold=False, align="l"):
    c.saveState(); c.setFillColor(color)
    f = "Helvetica-Bold" if bold else "Helvetica"
    c.setFont(f, size)
    xx = IN(x)
    yy = Y(y_top) - size  # baseline
    if align == "c":
        c.drawCentredString(xx, yy, s)
    elif align == "r":
        c.drawRightString(xx, yy, s)
    else:
        c.drawString(xx, yy, s)
    c.restoreState()


def wrap(x, y_top, w, s, size, color, bold=False, leading=None):
    """word-wrap within width w (inches), return next y_top."""
    c.setFont("Helvetica-Bold" if bold else "Helvetica", size)
    maxw = IN(w)
    words = s.split()
    line = ""
    lead = leading or size * 1.25
    yy = y_top
    for wd in words:
        test = (line + " " + wd).strip()
        if c.stringWidth(test, "Helvetica-Bold" if bold else "Helvetica", size) > maxw and line:
            text(x, yy, line, size, color, bold)
            yy += lead / 72
            line = wd
        else:
            line = test
    if line:
        text(x, yy, line, size, color, bold)
        yy += lead / 72
    return yy


def header(title, kicker=None):
    rect(0, 0, 13.333, 1.15, DARK)
    text(0.55, 0.45, title, 26, WHITE, True)
    if kicker:
        text(0.57, 0.92, kicker, 12, SUBT)


# ===== Slide 1 title
rect(0, 0, 13.333, 7.5, DARK)
rect(0, 2.55, 4.444, 0.06, BLUE)
rect(4.444, 2.55, 4.445, 0.06, GREEN)
rect(8.889, 2.55, 4.444, 0.06, RED)
text(0.8, 3.35, "State Machine & Per-State Control", 38, WHITE, True)
text(0.82, 4.55, "GreenPinkCamera Injectable Teardown", 22, SUBT)
text(0.82, 5.85, "Flexiv Rizon4  ·  6-state FSM  ·  primitive / Cartesian / joint-impedance control",
     14, GREY)
c.showPage()

# ===== Slide 2 two layers
header("The Controller Has Two Layers", "Describe every state with BOTH layers")
rrect(0.5, 1.4, 6.0, 5.6, fill=LIGHT)
text(0.7, 1.95, "Layer 1 - Flexiv RDK Control Mode", 16, DARK, True)
l1 = [("NRT_PRIMITIVE_EXECUTION", ACCENT,
       "Runs Flexiv primitives: MovePTP, MoveL, MoveC, InsertComp, FloatingCartesian, Contact, GraspComp, ZeroFTSensor"),
      ("NRT_CARTESIAN_MOTION_FORCE", GREEN,
       "Direct operational-space motion / force (grasp descent)"),
      ("NRT_JOINT_IMPEDANCE", BLUE,
       "Joint hold: SendJointPosition + SetJointImpedance (park arm rigidly during gripper-only actions)")]
y = 2.2
for name, col, desc in l1:
    rrect(0.7, y, 5.6, 1.35, fill=WHITE, stroke=col, sw=1.5)
    text(0.85, y + 0.32, name, 12, col, True)
    wrap(0.85, y + 0.62, 5.3, desc, 10.5, GREY)
    y += 1.5
rrect(6.85, 1.4, 6.0, 5.6, fill=LIGHT)
text(7.05, 1.95, "Layer 2 - Control Paradigm", 16, DARK, True)
text(7.05, 2.3, "the Force / Joint / Operational language", 11, GREY)
l2 = [("JOINT SPACE", BLUE, "MovePTP (joint space, % vel scale), joint-impedance hold, the cap twist"),
      ("OPERATIONAL / CARTESIAN", GREEN, "MoveL, MoveC - commanded in m/s"),
      ("FORCE / COMPLIANCE", RED, "InsertComp, FloatingCartesian, Contact, GraspComp, Cartesian impedance, gripper 80 N, vise 5 kg")]
y = 2.45
for name, col, desc in l2:
    rrect(7.05, y, 5.6, 1.35, fill=col)
    text(7.2, y + 0.35, name, 13, WHITE, True)
    wrap(7.2, y + 0.68, 5.3, desc, 10.5, WHITE)
    y += 1.5
c.showPage()

# ===== Slide 3 state machine
header("The State Machine", "6 states  ·  transition guards make it a real FSM")
states = ["Initialization", "Module Localization", "Pick Injectable",
          "Load Injectable", "Ultrasonic Cutting", "Component Disassembly"]
scols = [BLUE, BLUE, RED, RED, ACCENT, GREEN]
n = len(states); gap = 0.16; total_w = 13.333 - 1.0
bw = (total_w - gap * (n - 1)) / n
x = 0.5; ytop = 1.7; bh = 1.3
for i, (st, col) in enumerate(zip(states, scols)):
    rrect(x, ytop, bw, bh, fill=col)
    # center two-line label
    words = st.split()
    if len(words) > 1:
        text(x + bw / 2, ytop + 0.5, words[0], 12, WHITE, True, "c")
        text(x + bw / 2, ytop + 0.78, " ".join(words[1:]), 12, WHITE, True, "c")
    else:
        text(x + bw / 2, ytop + 0.62, st, 12, WHITE, True, "c")
    if i < n - 1:
        text(x + bw + gap / 2, ytop + 0.72, ">", 18, GREY, True, "c")
    x += bw + gap
rrect(0.5, 3.4, 12.33, 3.4, fill=LIGHT)
text(0.7, 3.85, "Transition guards (from recipe safety gates)", 15, DARK, True)
guards = [("> enter vise area", "Arduino busy=false, faulted=false, blade_on=false, homed"),
          ("> cut", "vise CLOSED, robot clear, rotary ~ 0"),
          ("> after cut", "CUT_HEIGHT=DONE, blade_on=false, x=z=0"),
          ("fault edge (any state)", "> STOP_ALL (fault handler)")]
gy = 4.35
for cond, detail in guards:
    text(0.8, gy, cond, 13, DARK, True)
    text(4.1, gy, detail, 12.5, GREY)
    gy += 0.62
c.showPage()

# ===== Slide 4 legend
header("Controller Legend", "Color code reused on every state slide")
legend = [(BLUE, "JOINT", "NRT_JOINT_IMPEDANCE", "MovePTP  ·  joint hold  ·  cap twist"),
          (GREEN, "OPERATIONAL", "NRT_CARTESIAN_MOTION_FORCE", "MoveL  ·  MoveC (m/s)"),
          (RED, "FORCE / COMPLIANCE", "force-aware primitives",
           "InsertComp · FloatingCartesian · Contact · GraspComp · gripper 80 N · vise 5 kg")]
y = 1.55
for col, name, mode, prims in legend:
    rrect(0.5, y, 0.5, 1.5, fill=col)
    rrect(1.1, y, 11.7, 1.5, fill=LIGHT)
    text(1.35, y + 0.55, name, 19, col, True)
    text(1.35, y + 1.05, mode, 11.5, GREY)
    text(6.0, y + 0.5, "Primitives:", 11.5, DARK, True)
    wrap(6.0, y + 0.82, 6.6, prims, 12.5, DARK)
    y += 1.72
c.showPage()

# ===== Slides 5-10 per state
state_data = [
    ("Initialization", "Bring robot, gripper and machine to a known safe state",
     [("gripper open", RED), ("MovePTP > Inter", BLUE), ("MovePTP > Home", BLUE)],
     [("Joint-space", BLUE), ("Gripper", RED)],
     "NRT_PRIMITIVE_EXECUTION", "MOVE_JNT_VEL_SCALE = 80%  ·  gripper 0.05 m/s"),
    ("Module Localization", "Re-localize the vise via ChArUco before any vise interaction",
     [("MovePTP > Vise-cali", BLUE), ("ChArUco detect (cali tag 1/2/3)", ACCENT),
      ("reload Vise.json", ACCENT), ("MovePTP > Home", BLUE)],
     [("Joint-space", BLUE), ("Vision", ACCENT)],
     "NRT_PRIMITIVE_EXECUTION", "hand-eye camera_tcp.yaml  ·  tag_01_to_vise_tcp.json"),
    ("Pick Injectable", "Camera-aligned approach + contact-guided adaptive grasp",
     [("MovePTP > Plate", BLUE), ("Align Injectable (MoveL)", GREEN),
      ("ZeroFTSensor > Contact > GraspComp", RED), ("MoveL lift +20 cm", GREEN)],
     [("Operational", GREEN), ("Force (contact grasp)", RED)],
     "PRIMITIVE_EXECUTION + CARTESIAN_MOTION_FORCE",
     "align 0.02 m/s  ·  contact 0.05 m/s  ·  grip 80 N"),
    ("Load Injectable", "SHOWCASE STATE - cycles through all three RDK modes",
     [("MovePTP > above_vise", BLUE), ("InsertComp (-Z > TCP X)", RED),
      ("FloatingCartesian y/z/rx while vise > 5 kg", RED),
      ("joint hold", BLUE), ("gripper release", RED), ("MoveL retreat", GREEN)],
     [("Force/compliance", RED), ("Joint hold", BLUE), ("Operational", GREEN)],
     "ALL THREE RDK MODES",
     "INSERTCOMP_INSERT_VEL = 0.02  ·  maxContactForce  ·  float maxVel 0.2 m/s"),
    ("Ultrasonic Cutting", "Machine cuts while robot waits clear; then grip + twist off cap",
     [("Arduino CUT_HEIGHT (robot idle/clear)", ACCENT), ("MoveL down", GREEN),
      ("FloatingCartesian y/z/rx", RED), ("gripper 80 N", RED),
      ("twist about TCP X  0>+7>-10 deg", BLUE), ("MoveL lift", GREEN)],
     [("Machine", ACCENT), ("Force-float", RED), ("Joint twist", BLUE)],
     "NRT_PRIMITIVE_EXECUTION", "CUT z=134.2, x=110.5, deg=360  ·  twist scale 20%"),
    ("Component Disassembly", "Fast-mode pick-and-place of spring, plastic, shell & glass",
     [("MovePTP transit (xN)", BLUE), ("MoveL descend / lift 0.5 m/s", GREEN),
      ("gripper 80 N", RED), ("Arduino OPEN_VISE", ACCENT), ("MoveC dump arc", GREEN)],
     [("Joint transit", BLUE), ("Operational", GREEN), ("Force-grip", RED)],
     "NRT_PRIMITIVE_EXECUTION", "FRAME5_CARTESIAN_VEL = 0.5 m/s  ·  dump 0.03 m/s"),
]
for idx, (title, kicker, seq, tags, mode, params) in enumerate(state_data, 1):
    header(f"State {idx} - {title}", kicker)
    text(0.55, 1.55, "CONTROL:", 11, GREY, True)
    cx = 1.7
    for tname, tc in tags:
        w = 0.35 + 0.085 * len(tname)
        rrect(cx, 1.26, w, 0.4, fill=tc, r=6)
        text(cx + w / 2, 1.50, tname, 11, WHITE, True, "c")
        cx += w + 0.12
    text(0.55, 2.2, "Primitive / action sequence", 14, DARK, True)
    y = 2.4
    for i, (label, col) in enumerate(seq):
        rrect(0.7, y, 7.4, 0.6, fill=WHITE, stroke=col, sw=1.6, r=6)
        rect(0.7, y, 0.12, 0.6, col)
        text(1.0, y + 0.4, f"{i+1}.  {label}", 12.5, DARK)
        y += 0.7
    rrect(8.45, 2.4, 4.4, 2.0, fill=LIGHT)
    text(8.65, 2.78, "RDK control mode", 12, GREY, True)
    wrap(8.65, 3.15, 4.0, mode, 12.5, DARK, True, leading=16)
    rrect(8.45, 4.6, 4.4, 2.0, fill=LIGHT)
    text(8.65, 4.98, "Defining parameters", 12, GREY, True)
    wrap(8.65, 5.35, 4.0, params, 12, DARK, leading=16)
    c.showPage()

# ===== Slide 11 takeaways
rect(0, 0, 13.333, 7.5, DARK)
text(0.8, 1.35, "Two Things To Remember", 30, WHITE, True)
rrect(0.8, 1.7, 11.7, 2.2, fill=PANEL)
text(1.1, 2.4, "1 . Color-code by control paradigm", 20, LBLUE, True)
wrap(1.1, 2.95, 11.1,
     "Blue = joint, green = operational, red = force - consistent across the FSM and "
     "every state slide. Pick & Load read as force-rich; Disassembly is mostly kinematic.",
     15, TXT2, leading=20)
rrect(0.8, 4.1, 11.7, 2.4, fill=PANEL)
text(1.1, 4.8, "2 . Load Injectable is the showcase state", 20, LRED, True)
wrap(1.1, 5.35, 11.1,
     "It is the only state that cycles through all three RDK modes: compliant InsertComp "
     "> floating force > joint-impedance hold > Cartesian retreat. This is your "
     "'we understand control architecture' slide.",
     15, TXT2, leading=20)
c.showPage()

c.save()
print("saved state_machine_controller_slides.pdf")
