def cut at height (z, x, deg):
define x near cut position of 100mm
use supplied x as the cut position
use supplied deg as the cut rotation

check if vise is closed (force >3 kg), if not throw error
if yes send ack and set machine state busy
Move z to height zmm
move to x near cut position of 100mm
turn on the blade
move x to supplied cut position, slowly
rotate the stage by supplied degrees
back off x to near cut position of 100mm, slowly
turn off the blade
rotate the stage in the opposite direction back to 0
move x to position 0mm
send ack that cut is done







recipe for inj 1
cut at 110mm, too low, the whole spring came off
maybe 100?
100 is still a bit low and the springs shoots off
maybe 96, 96 the spring is still stuck, maybe higher

let's try 86 = latch place, ok not springing off, but latch on a bit tight

try 89, depth 118


cut at 130mm, too high, the plastic stuck in it, may be 133?
135 is too low

try 133


cut at 150mm, maybe don't need this cut? can simply dump




first cut:
cut_height z_mm=91 x_mm=111 deg=359


second cut:
cut_height z_mm=134 x_mm=111 deg=360