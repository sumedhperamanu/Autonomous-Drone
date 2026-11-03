# 5" and 7" sims

Both drone options in Gazebo, so you can fly them yourself and see the numbers from Day 3-4.
We're going with the 7" on 4S. The 5" and the 7" on 6S are here so you can compare.

Everything runs on our locked stack (PX4 v1.17.0 + Gazebo Harmonic, see `docs/STACK.md`).
Results and how I got them: `docs/p1/day3-4-results.md`.

| make target | What it is | Motors at hover |
| --- | --- | --- |
| `gz_seveninch_4s` | 7" on 4S, 2807 1700 KV, 4S 5000. The one we're buying | about 0.39 |
| `gz_seveninch_lidar_flow` | Same 7" with the phase 1 sensors: 360 lidar, downward flow, no GPS | about 0.39 |
| `gz_fiveinch` | 5" from Day 2, with the fixed inertia and gains | about 0.35 |
| `gz_fiveinch_lidar_flow` | 5" with the phase 1 sensors | about 0.35 |
| `gz_seveninch` | 7" on 6S. We dropped this one, it's just for comparing | about 0.33 |

## Setup (once)

You need Docker and WSL Ubuntu (or just Ubuntu). Do these in the Ubuntu terminal.

1. Get the branch:

   ```bash
   git clone -b sim-5-in-7-in https://github.com/sumedhperamanu/Autonomous-Drone.git ~/Autonomous-Drone
   cd ~/Autonomous-Drone
   ```

   If you already have the repo: `git fetch origin && git checkout sim-5-in-7-in`.
   Clone it inside WSL, not in a Windows folder, or the scripts get Windows line endings.

2. Build the team image. This is P2's dev container and takes a while the first time (30+ min):

   ```bash
   docker build -t drone-dev .devcontainer
   ```

3. Start a container with the Gazebo window working and the repo mounted at `/workspace`:

   ```bash
   docker run -it --name drone-sim -u ubuntu --network=host --shm-size=2g \
     -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR \
     -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
     -v ~/Autonomous-Drone:/workspace -w /workspace \
     drone-dev bash
   ```

   After the first time, use `docker start -ai drone-sim` to get back in.

4. In the container, copy the drones into PX4:

   ```bash
   bash sim/install.sh
   ```

   It finds PX4 by itself and prints the next command. Run it again any time you pull new changes.

Other ways in: VS Code's "Reopen in Container" works too (skip steps 2 and 3 and use its terminal).
If you still have the `drone-sim:px4-1.17` container from my 5" test, that works as well. PX4 is at `~/PX4-Autopilot`
in that one instead of `/opt/PX4-Autopilot`, so swap the path in the commands below.

## Every time after that: one paste in PowerShell

Once setup is done, open PowerShell (with Docker Desktop running), paste one of these, and press Enter.

**Sim only:**

```powershell
docker restart -t 0 drone-sim
docker exec -it drone-sim bash -lc "bash ~/run_sim.sh gz_seveninch_4s"
```

**Sim and QGroundControl, already connected:**

```powershell
docker restart -t 0 drone-sim
$qgc = Get-ChildItem "C:\Program Files\QGroundControl*" -Recurse -Filter QGroundControl.exe -ErrorAction SilentlyContinue | Select-Object -First 1
if (Get-Process QGroundControl -ErrorAction SilentlyContinue) { } elseif ($qgc) { explorer.exe $qgc.FullName } else { Write-Host "QGroundControl not found, open it by hand" }
docker exec -it drone-sim bash -lc "bash ~/run_sim.sh gz_seveninch_4s"
```

Gazebo opens and the `pxh>` prompt shows up in the same window. With the second one, QGC opens too (unless it's
already open) and shows the drone about 10 s after `Ready for takeoff!`. Type `shutdown` at `pxh>` when you're done.

Things that tripped me up:

- Paste into a fresh PowerShell window (the prompt starts with `PS C:\`), not at `pxh>`. If a sim is already
  running, `shutdown` it first, or just open QGC from the Start menu and it connects on its own.
- If nothing happens after pasting, press Enter. Don't paste again, or the two copies get glued together.
- `docker restart` clears out any sim left running from last time, so it always starts clean.
- QGC is opened through `explorer.exe` on purpose. If PowerShell opens it directly, QGC prints its log into this
  window and buries the PX4 output.
- If your container has another name (mine is `fiveinch`), swap it in.

For a different drone, change the last line:

```powershell
docker exec -it drone-sim bash -lc "bash ~/run_sim.sh gz_fiveinch"                    # 5"
docker exec -it drone-sim bash -lc "bash ~/run_sim.sh gz_seveninch_lidar_flow walls"  # 7" with sensors, room with walls
```

`~/run_sim.sh` is put there by `install.sh` and runs the same `make px4_sitl ...` as below. It always tries to link
PX4 to QGC on your PC. If QGC isn't open nothing happens, and you can ignore the `QGroundControl link` line.

**If Gazebo feels slow:** look at the real time factor in the bottom right of the Gazebo window. Around 100% is fine.
Lower than that means the PC can't keep up, and the sim plays in slow motion. The physics and the numbers are still
right (PX4 waits for Gazebo), it just takes longer. Closing QGC and other heavy apps helps. For the step tests you
don't need the window at all (`sim/tests/` runs headless).

## Commands

### PowerShell (Windows)

| Command | What it does |
| --- | --- |
| `docker restart -t 0 drone-sim` | Starts the container, or resets it if a sim is stuck |
| `docker exec -it drone-sim bash -lc "bash ~/run_sim.sh gz_seveninch_4s"` | Starts a sim (target, then world) |
| `docker exec -it drone-sim bash` | A shell in the container without starting a sim. `exit` to leave |
| `docker stop drone-sim` | Stops the container when you're done |
| `docker ps -a` | Lists containers and whether they're running |
| `docker cp file.zip drone-sim:/home/ubuntu/` | Copies a file from Windows into the container |

### run_sim.sh

`bash ~/run_sim.sh <target> <world>`. Both are optional (default `gz_seveninch_4s` in the `default` world).

- Targets: `gz_seveninch_4s`, `gz_seveninch_lidar_flow`, `gz_fiveinch`, `gz_fiveinch_lidar_flow`, `gz_seveninch`
  (the table at the top).
- Worlds: `default` (flat ground) or `walls` (a room, for the lidar). PX4 has more in
  `Tools/simulation/gz/worlds/`. For wind use `wind.py` (see Wind below), not the `windy` world.
- `QGC_IP=none bash ~/run_sim.sh ...` skips the QGC link. `HEADLESS=1 bash ~/run_sim.sh ...` runs with no Gazebo
  window (fastest, but you can't watch it). From PowerShell these go inside the quotes:
  `docker exec -it drone-sim bash -lc "HEADLESS=1 bash ~/run_sim.sh gz_seveninch_4s"`.

### At pxh>

Wait for `Ready for takeoff!` first. Press Tab to autocomplete, or type a command on its own to see its options.

| Command | What it does |
| --- | --- |
| `commander takeoff` | Arms and climbs to 2.5 m, then holds there |
| `commander land` | Lands where it is and disarms |
| `commander mode auto:rtl` | Flies back to the takeoff spot and lands |
| `commander mode auto:loiter` | Holds position |
| `commander status` | Mode, armed or not, and any failsafe |
| `commander disarm -f` | Kills the motors right away (it drops if it's flying) |
| `listener actuator_motors` | The 4 motor outputs, 0 to 1. About 0.39 at hover on the 7" |
| `listener vehicle_local_position` | Position and speed in m. z is negative going up, so 2.5 m up is z = -2.5 |
| `listener vehicle_attitude` | Orientation (as a quaternion) |
| `listener battery_status` | Battery voltage and percent left |
| `listener obstacle_distance` | Lidar, 72 distances in cm (sensor versions only) |
| `listener distance_sensor` | Downward range in m (sensor versions only) |
| `listener sensor_optical_flow` | Flow quality, 255 is best (sensor versions only) |
| `listener <topic> -n 5` | Prints 5 messages instead of 1 |
| `uorb top` | Every topic and how fast it's updating. `q` to quit |
| `param show MC_ROLLRATE_K` | Shows a parameter. `param show MC_*` shows a whole group |
| `param set MC_ROLLRATE_K 0.5` | Changes one. It's saved and still there next run |
| `param reset_all` | Puts every parameter back to the airframe defaults |
| `top` | CPU use per module. `q` to quit |
| `shutdown` | Quits PX4 and Gazebo |

pxh can take off, land, hold and come home, but it can't steer. To fly it around, use QGC's virtual joystick (below)
or ROS 2 offboard (P2's side).

A failsafe you can try: `param set COM_LOW_BAT_ACT 3` and `param set SIM_BAT_MIN_PCT 3`, then `commander takeoff`.
The fake battery drains about 1% every 0.6 s while armed, so within a minute the drone should warn at 15%, fly home
at 7% and land at 5% (PX4's default thresholds). Run `param reset_all` afterwards.

## Fly it

```bash
cd /opt/PX4-Autopilot
make px4_sitl gz_seveninch_4s
```

The first build takes 5 to 10 minutes. Gazebo opens with the drone on the ground. Wait for `Ready for takeoff!`,
then at the `pxh>` prompt type one line at a time:

```
commander takeoff
listener actuator_motors
commander land
```

**Pass:** it climbs to about 2.5 m and sits still, and the 4 motor values are close to each other and close to the
table above (about 0.39 for the 7" on 4S). Type `shutdown` to quit.

For the 5", same thing with `make px4_sitl gz_fiveinch` (motors about 0.35).
The 7" needs more throttle because it's heavier, but it hovers longer because big props make lift more efficiently.

## Sensor version

```bash
PX4_GZ_WORLD=walls make px4_sitl gz_seveninch_lidar_flow
```

At `pxh>`:

```
listener obstacle_distance
listener distance_sensor
listener sensor_optical_flow
```

**Pass:** 72 lidar distances with the walls at about 450 cm, ground distance about 0.07 m, flow quality 255.
That's what the 5" sensor version gave on Oct 29. The 7" sensor version is new and nobody has flown it yet, so if
something looks wrong, send me the pxh output.

## Wind

`~/wind.py` blows gusty wind at whichever drone is flying, with numbers for Waterloo. Start the sim and take off
(`commander takeoff`), then open a second PowerShell window and run one of these:

```powershell
docker exec -it drone-sim python3 ~/wind.py typical   # 5 m/s (18 km/h), gusts to about 8 (29 km/h)
docker exec -it drone-sim python3 ~/wind.py windy     # 9 m/s (32 km/h), gusts to about 15 (54 km/h)
docker exec -it drone-sim python3 ~/wind.py storm     # 13 m/s (47 km/h), gusts to about 20 (72 km/h)
docker exec -it drone-sim python3 ~/wind.py 7 12      # your own: mean and gust in m/s
```

Ctrl+C stops it. If that window got closed and the wind stayed on, run `python3 ~/wind.py off` the same way.
`--from 300` changes where it comes from (compass degrees, default 250 = WSW, the usual direction here), and
`--steady` turns the gusts off. Watch it lean into the wind in Gazebo, or at `pxh>` run
`listener vehicle_local_position` to see how far it gets pushed.

Where the numbers come from: Waterloo averages about 15 to 20 km/h of wind, more from fall to spring. On Nov 3 the
airport had 18 km/h gusting to 27, which is what `typical` is. Those are weather station numbers, 10 m up. We fly at
3 m or less, where it's usually a bit calmer, so these err on the harsh side.

How it pushes the drone: it sets Gazebo's world wind, and the motor model turns that into rotor drag using the air
speed relative to the drone. The push drops when the drone drifts with the wind and grows when the motors spin up.
I set the drag coefficient to cover the props plus the frame, battery and electronics (0.04 m^2 of drag area on the
7", 0.025 m^2 on the 5"), matched at 10 m/s. Gazebo's drag is linear and real drag grows with speed squared, so
it's a bit too harsh below 10 m/s and a bit too gentle in the biggest gusts. The gusts are random turbulence.

What I got on Nov 3 (headless, 45 s each, GPS position hold at 2.5 m, baseline gains):

| | 7" typical | 7" windy | 5" typical | 5" windy |
| --- | --- | --- | --- | --- |
| Lean, average / peak | 11 / 21 deg | 17 / 33 deg | 9 / 22 deg | 16 / 31 deg |
| Pushed off its spot, peak | 1.3 m | 1.6 m | 0.9 m | 1.8 m |
| Height while holding 2.5 m | 2.45 to 2.63 m | 2.38 to 2.70 m | 2.34 to 2.56 m | 2.34 to 2.64 m |
| Highest motor output | 0.42 | 0.42 | 0.38 | 0.41 |

- Both drones hold on through a windy day with plenty of thrust left. The motors never went past 0.42. The limit
  is PX4's 45 deg maximum lean (`MPC_TILTMAX_AIR`), not the motors. Try `storm` to see where they give up.
- Gusts push them 1 to 2 m off their spot. P3's obstacle margin is 0.5 m, so on a windy day the drone can drift
  into it. The position hold gains are still PX4 defaults (`MPC_XY_P`, `MPC_XY_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`),
  and tuning those is how we tighten this.
- These runs hold position with GPS. The real drone uses optical flow, which will probably hold worse in gusts.
- Not in the sim: wind shadow and swirl around buildings and trees, and the wind messing with the flow sensor.

## QGroundControl (optional)

You don't need QGC to fly the sim, but it lets you fly with a virtual joystick and load parameter files.

1. Use the "Sim and QGroundControl" paste above. run_sim.sh links PX4 to QGC by itself, and QGC should show the
   drone within about 10 s of `Ready for takeoff!`.
2. If it doesn't, add the link by hand. Find the Windows IP with
   `docker exec drone-sim getent hosts host.docker.internal` (Docker Desktop), or from WSL with
   `ip route | awk '/default/ {print $3}'` if Docker runs inside WSL. Then at `pxh>`:

   ```
   mavlink start -u 14561 -o 14550 -t <that IP> -r 4000000 -x
   ```

   Windows may ask to let QGC through the firewall the first time. Say yes.
3. In QGC turn on the virtual joystick (Application Settings > Fly View > Virtual Joystick), take off with the
   slider, switch to Position mode, and fly around. On `gz_seveninch_lidar_flow` with the walls world, fly slowly at a
   wall: it should stop about 1 m away (Collision Prevention only works in Position mode).
4. To load our baseline parameters: Vehicle Setup > Parameters > Tools > Load from file, pick
   `params/sim_seveninch_baseline.params` (or the 5" one), then reboot the vehicle. QGC will flag `MPC_XY_CRUISE`
   and `MPC_VEL_MANUAL` as below their minimum. That's expected.

## Step tests

These fly the same test every time (altitude step, 10 deg roll and pitch steps) and print rise time, overshoot,
hover output, wobble and lag. Headless, start to finish:

```bash
PITCH=1 PLOT=7in.png bash sim/tests/run_case.sh 22054 gz_seveninch_4s
PITCH=1 PLOT=5in.png bash sim/tests/run_case.sh 22050 gz_fiveinch
```

How to read the output and what passes: `sim/tests/README.md`.

## What's where

```
sim/install.sh      copies both drones into PX4
sim/run.sh          starts a sim and links it to QGC (install.sh copies it to ~/run_sim.sh)
sim/wind.py         gusty wind for a running sim (install.sh copies it to ~/wind.py)
sim/fiveinch/       5" model, airframes 22050 and 22051, and its numbers (README)
sim/seveninch/      7" model, airframes 22052 (6S), 22053 (4S + sensors), 22054 (4S), and its numbers
sim/tests/          step test and log analysis scripts
sim/calc/           motor model, inertia and param file scripts
params/             baseline parameter files for sim and for the real 7" first flight
docs/p1/            Day 3-4 results and step response plots
```

## If something goes wrong

- **No Gazebo window.** Run `echo $DISPLAY` in the container. If it's empty, the docker run line was missing the
  display parts. Running `wsl --update` in PowerShell fixes most WSL window problems.
- **`Couldn't find PX4`.** Give the path: `bash sim/install.sh ~/PX4-Autopilot`.
- **make doesn't know `gz_seveninch_4s`.** Run `bash sim/install.sh` again, then make again.
- **`$'\r': command not found`.** The files have Windows line endings. Clone inside WSL (step 1).
- **Shakes or flips right at takeoff.** Check `param show MC_ROLLRATE_K` at pxh. It should be 0.4 on the 5" and 0.5
  on the 7". If it's 1.0 you have old saved parameters: quit, run
  `rm /opt/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters*.bson`, and start again.

## What the sim can't tell us

- The real drone will need lower gains. Motor delay, vibration and frame flex aren't in the model. The first real
  flight uses `params/hw_seveninch_first_flight.params` (board default gains), and we tune from there.
- No battery sag or ground effect. Wind is an approximation (see Wind).
- The 7" motor numbers are from my motor model. eCalc agrees within 15% on thrust and hover throttle.
