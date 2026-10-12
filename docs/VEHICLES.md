# Vehicles: the boat, the car, the drone, and the keys

One runtime body (`sim/Vessel.h`) consumes any spec (`sim/VesselSpec.h`); a vehicle is a spec, and
a new kind of physics is a new element kind. The scene declares which vehicles exist and where
(`entities[]`), the frame loop steps them all every frame, and the helm is on ONE of them at a
time. This file is the player's side of that: what the keys do, and the laws behind them in a
sentence each.

## The keys

| key | every vehicle |
|---|---|
| **T** / **Y** | cycle the helm forward / back through the scene's vehicles, in scene order |
| **F** | free the camera from the vehicle (it keeps its controls); F again returns |
| **W / S** | the first channel: both throttles ahead / astern (boat), drive / reverse (car), forward / back (drone) |
| **A / D** | the steer channel: the outboards (boat), the front wheels (car), yaw rate (drone) |
| **Q / E** | the split channel: the levers apart, a pivot (boat); strafe left / right (drone); nothing on the car |
| **SHIFT / CTRL** | the tilt channel: trim out / in (boat, it stays where you leave it); the brake, SHIFT only (car, it releases); climb / descend (drone, it releases) |

The helm reads each vehicle's own joints for the channel's limits and rates: the first jointed
element's are the steer's (the RHIB's outboard swings to 0.6 rad at 1.2 rad/s; the car's kingpin
to 0.55 rad at 2.5 rad/s), the first tilted element's are the tilt's, with a return rate (0 for a
trim pump, which stays put; a pedal or a lever comes back). D is starboard on every vehicle; the
sign the physics needs (an outboard kicks the stern the other way, a wheel points where it goes)
is the element's business, not the helmsman's.

## One body, three forms (parts plus forms)

The scenes declare ONE entity, `vehicle`, with three `forms`: the boat, the drone, the car. A spec
is a list of parts; a form is how much of each part is engaged and the body's own numbers in that
form (`sim/VesselSpec.h` Form). The three vehicles have no parts in common, so their engagements
are 0 or 1; a transformer later is the same table with parts in common and mounts moving between
forms along their screws (docs/ALGEBRA.md, pga). **T / Y take the next / previous form.** Taking a
form moves the body's origin to that form's CG and carries the pose by the same shift, so every
part stands where it stood; the momenta are rested and the seat law (below) puts the new form where
it belongs. The parts a form does not engage weigh nothing, ask the water nothing and are not drawn.

## The recovery drone

The drone is the recovery form. Flip the boat or roll the car: press **T** to take the drone form
and it appears six metres over where you were, upright, at your heading. Fly it
(W/S/Q/E, SHIFT/CTRL up and down, A/D turn) to wherever you want the vehicle to be, then **T** (or
**Y**) back to the boat or the car: it takes that form where the drone is, upright, facing the
drone's way, and is set down on whatever is there -- the boat at its draught in the water, the car on its tyres on the
ground, by the one seat law below. Between two ordinary vehicles T/Y only moves the helm; nothing
is moved.

The drone is a velocity servo and an attitude servo with one time constant (0.35 s) and no fluid:
it asks the water nothing, so it flies the same over the town and the sea. Its numbers
(12 m/s across, 5 m/s up and down, a turn in five seconds) are the spec's and all ASSUMED.

## The seat: the top of the stack

Every vehicle is set down, at its spawn and after a recovery, with its lowest point
(`Vessel::KeelY`: the keel of the hull's stations, or the tyres at full droop) at its
`seatHeight` above the surface it rides. That surface is the top of the water-over-bed stack,
`max(water, bed)` -- which is exactly what "depth <= 0 is dry" already said -- so one law seats a
boat on the sea, a boat on a beach, a car on a road and a drone six metres up. A boat's seat is
minus its draught (it floats where it floats; that number is what the set-down always used), a
car's is zero (it lands on its tyres and settles into its springs), the drone's is its hover
height.

## The ground, and beaching

The bed is a second surface. Every sample the water answers already carries it (`bedNavd`), and
the hull's own station outline points -- the ones clipped against the water for buoyancy -- are
pushed back along the bed's normal where they fall below it: a spring the hull's own weight
sizes (1.5 cm of settle on the keel points), damped, with Coulomb friction. The friction is a
GRIP: each point keeps the ground point it touched and a tangential spring holds it there up to
`mu N`, past which it slides at exactly `mu N` (a velocity law cannot hold a body at rest on a
slope at 60 Hz; it creeps a step a frame). Run the boat at a beach and the water clip sees less
and less hull, buoyancy falls, the bed carries what it dropped, and friction holds it: that is a
beaching, and nothing in it reads the depth to decide anything. Back off with astern thrust on
the same law. The bed's normal is one plane per hull per step, the secant over the hull's length.

## The car: `car.baja`

A 2003 Subaru Baja, its published figures (wheelbase 2.65 m, 1,610 kg curb, P225/60R16) and its
wheel positions measured from the reference model (`D:\Assets\subaru-baja\source\Subaru Baja.glb`),
lifted 0.25 m and sprung long and soft so it drives like the Warthog: 0.35 m of travel on a 1.2 Hz
spring at half-critical damping, 8 kN of shove over four driven wheels falling to nothing at
32 m/s. All of the Warthog in it is declared in the spec and says ASSUMED.

Each wheel is a strut cast down onto the SAME bed the hull beaches on (the raycast vehicle, which
is what the Warthog is): a progressive spring `k x (1 + (x/travel)^2)` whose cube is its own bump
stop, a damper, and a tyre at its foot. The tyre's contact patch grips a ground point: along the
rolling direction the brake pedal (SHIFT) scales that grip, so a braked car stands on a hill;
across it the patch is the carcass spring over a relaxation length (0.3 m), relaxing as the patch
rolls through -- at rest a spring that holds a parked car across a camber, rolling exactly the
slip-angle law as its limit. Drive, rolling resistance, brake and lateral all go through one
friction cone, `min(1, mu N / |F|)`. The front wheels steer about their kingpin lines
(`Attach::Posed`, the same primitive as a steered outboard). Under water the wheels find the same
bed; nothing floats the car (a drag box slows it at axle height), and the drone is the recovery.

Gates (`--selftest`): at rest every wheel carries its axle's static share to 0.7 % at the
progressive spring's compression to 0.6 %; a 20 degree hill, brake on, drifts 0.0000 m in 10 s;
full throttle for 5 s reaches 15.9 m/s against the drive law's own 16.6 (rolling and air below
it), no yaw, four wheels down; a step steer while coasting meets Ackermann's `tan(delta)/L`
(neutral steer by construction: the cornering stiffness is the load's).

The scenes (`haulover_portal.json`, `haulover_city.json`) spawn the one vehicle as the boat at the
entrance; T takes the drone form, T again the car. To drive the car, fly the drone over land
first (the south-bank beach is 500 m west), then T: the car is set down on the sand.
