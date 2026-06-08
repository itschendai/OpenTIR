# State Machine & Per-State Control — GreenPinkCamera Injectable Teardown

Slide outline for the robotics presentation: "State Machine and Controller used
for each State". Recipe: `recipe/GreenPinkCameraFast/recipe.py`.

> Note: there are **6** phases (states), one content slide each.

---

## The key framing: the controller has two layers

Every state is described by **both** layers. Say both on each slide.

### Layer 1 — Flexiv RDK control mode (the low-level loop)

| Mode | Purpose | Used by |
| --- | --- | --- |
| `NRT_PRIMITIVE_EXECUTION` | Runs Flexiv primitives | MovePTP, MoveL, MoveC, InsertComp, FloatingCartesian, Contact, GraspComp, ZeroFTSensor |
| `NRT_CARTESIAN_MOTION_FORCE` | Direct operational-space motion/force | Grasp descent |
| `NRT_JOINT_IMPEDANCE` | Joint hold (`SendJointPosition` + `SetJointImpedance`) | Parking arm rigidly during gripper-only actions |

### Layer 2 — Control paradigm (Force / Joint / Operational)

| Paradigm | What runs in it |
| --- | --- |
| **Joint space** | MovePTP (planned in joint space, % vel scale), joint-impedance hold, the cap twist |
| **Operational / Cartesian space** | MoveL, MoveC (commanded in m/s) |
| **Force / compliance** | InsertComp, FloatingCartesian, Contact, GraspComp, Cartesian impedance, gripper force-target (80 N), vise close-to-force (5 kg) |

---

## Suggested deck structure (~9 slides)

### Slide 1 — Title
"State Machine & Per-State Control — GreenPinkCamera Injectable Teardown"

### Slide 2 — The State Machine (anchor slide)
Horizontal flow diagram of the 6 states with transition guards.

```
Initialization → Module Localization → Pick Injectable → Load Injectable → Ultrasonic Cutting → Component Disassembly
```

Transition guards (from the recipe safety gates — makes it a real FSM, not a script):
- → enter vise area: `Arduino busy=false, faulted=false, blade_on=false, homed`
- → cut: `vise CLOSED, robot clear, rotary ≈ 0`
- → after cut: `CUT_HEIGHT=DONE, blade_on=false, x=z=0`
- fault edge from any state → `STOP_ALL` (fault handler)

### Slide 3 — Controller Legend (reusable key)
Define the 3 control paradigms ↔ 3 RDK modes ↔ primitives, with a color code.
Reuse those colors on every per-state slide.

Suggested color code: **blue = joint**, **green = operational**, **red = force**.

### Slides 4–9 — One per state
Same layout each time: *Goal · Primitives/sequence · Control paradigm · RDK mode · Key params*.

---

## Per-state content

| State | Primitives (in order) | Control paradigm | RDK mode | Defining params |
| --- | --- | --- | --- | --- |
| **Initialization** | gripper open; `MovePTP`→Inter→Home | Joint-space + gripper | `NRT_PRIMITIVE_EXECUTION` | `MOVE_JNT_VEL_SCALE=80%`, gripper 0.05 m/s |
| **Module Localization** | `MovePTP`→Vise-cali; ChArUco detect (`cali tag 1/2/3`); reload `Vise.json`→Home | Joint-space + vision (no closed-loop force) | `NRT_PRIMITIVE_EXECUTION` | hand-eye `camera_tcp.yaml`, `tag_01_to_vise_tcp.json` |
| **Pick Injectable** | `MovePTP`→Plate; `Align Injectable` (`MoveL`); `ZeroFTSensor`→`Contact`→`GraspComp`; `MoveL` lift +20 cm | Operational + **force** (contact-guided grasp) | `NRT_PRIMITIVE_EXECUTION` + `NRT_CARTESIAN_MOTION_FORCE` | align 0.02 m/s, contact 0.05 m/s, grip 80 N |
| **Load Injectable** | `MovePTP` above_vise; **`InsertComp`** (−Z→TCP X); **`FloatingCartesian`** y/z/rx while vise→5 kg; **joint hold**; gripper release; `MoveL` retreat | **Force/compliance** → joint hold → operational | all three modes | `INSERTCOMP_INSERT_VEL=0.02`, maxContactForce, float maxVel 0.2 m/s |
| **Ultrasonic Cutting** | Arduino `CUT_HEIGHT` (robot idle/clear); `MoveL` down; `FloatingCartesian` y/z/rx; gripper 80 N; **twist about TCP X** (`MovePTP` 0→+7→−10°); `MoveL` lift | Machine + force-float + **joint-space twist** | `NRT_PRIMITIVE_EXECUTION` | `CUT z=134.2, x=110.5, deg=360`; twist scale 20% |
| **Component Disassembly** | repeated `MovePTP` transit + `MoveL` descend/lift (fast 0.5 m/s); gripper 80 N; Arduino `OPEN_VISE`; **`MoveC`** dump arc | Joint transit + operational + force-grip | `NRT_PRIMITIVE_EXECUTION` | `FRAME5_CARTESIAN_VEL=0.5`, dump 0.03 m/s |

---

## Two design tips

- **Color-code by control paradigm** (blue=joint, green=operational, red=force)
  and keep it consistent across the FSM diagram and every state slide. The
  audience instantly sees *Pick* and *Load* are force-rich while *Disassembly*
  is mostly kinematic.
- **Call out Load Injectable as the showcase state** — it is the only one that
  cycles through *all three* RDK modes (compliant InsertComp → floating force →
  joint hold → Cartesian retreat). That is the "we understand control
  architecture" slide.
