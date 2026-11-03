# Step tests

Scripted test flights in Gazebo and the log analysis behind the Day 4 numbers. Every flight is the same, so if a
number changes it's because of the drone or its gains, not how someone flew it.

The flight: offboard take-off to 2 m, hover 10 s, altitude step 2 -> 3 -> 2 m, a 10 deg roll step, a 10 deg pitch
step (with `--pitch` or `PITCH=1`), then land.

The scripts need `pymavlink`, `pyulog`, `numpy` and `matplotlib`. The team dev image already has them (PX4's setup
installs them). PX4 has to be built once first (`make px4_sitl gz_fiveinch`).

## Run

Headless, start to finish. This stops any sim that's already running:

```bash
bash sim/tests/run_case.sh 22054 gz_seveninch_4s                    # 7" on 4S
PITCH=1 PLOT=7in.png bash sim/tests/run_case.sh 22054 gz_seveninch_4s   # with the pitch step and a plot
bash sim/tests/run_case.sh 22050 gz_fiveinch                        # 5"
bash sim/tests/run_case.sh 22054 gz_seveninch_4s MC_ROLLRATE_K=0.7 MC_PITCHRATE_K=0.7   # try other gains
```

With the Gazebo window open instead (sim already running from `make px4_sitl ...`, second terminal):

```bash
python3 sim/tests/step_tests.py --pitch
python3 sim/tests/analyze_steps.py $(ls -t /opt/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg | head -1)
```

Gain sweep, several gains in a row (results in `/tmp/sweep_<model>.txt`):

```bash
bash sim/tests/sweep.sh 22054 gz_seveninch_4s 0.3 0.5 0.7 1.0
```

Set `PX4_DIR=...` if PX4 isn't at `~/PX4-Autopilot` or `/opt/PX4-Autopilot`.

| File | What it does |
| --- | --- |
| `run_case.sh` | One flight end to end: start the sim headless, fly, stop, analyse |
| `sim_headless.sh` | Starts PX4 + Gazebo with no window and fresh parameters |
| `step_tests.py` | Flies the test over MAVLink |
| `analyze_steps.py` | Reads the log: rise time, overshoot, settling, hover motor output, wobble, lag |
| `sweep.sh` | `run_case.sh` at several rate gains |
| `summarize_logs.py` | One line per log, for comparing runs |
| `final_batch.sh` | The Day 4 final set of flights |

## Reading the numbers

| Number | What it tells you | Pass |
| --- | --- | --- |
| Hover motor output | Thrust needed to hover. Should match eCalc's hover throttle | under 0.5 (team spec) |
| Altitude step rise / overshoot / settle | Mostly `MPC_Z_P`, not the rate gain | overshoot under 10%, settle under 5 s |
| Roll and pitch step rise / overshoot | Mostly `MC_ROLL_P` / `MC_PITCH_P` | rise under 1 s, overshoot under 15% |
| Rate-tracking lag | How far the real roll rate trails the command. Best single number for the rate gain | under 60 ms |
| Hover wobble | Roll-rate jitter in a still hover. This is where shaking shows up first | under 1 deg/s |

Some things I learned doing this:

- Raise `MC_ROLLRATE_K` and `MC_PITCHRATE_K` together until the hover starts to shake (wobble jumps from under 1 to
  25+ deg/s), then use about half. A real drone shakes at a lower gain than the model.
- Rate-tracking lag changes by about 6 ms between identical runs, so 40 vs 46 ms isn't a real difference.
- During the roll and pitch steps there's no position hold, so the drone drifts a couple of metres sideways. Fine in sim.
  On a real drone don't run these without at least 5 m of space all round.
- Before Nov 3, `step_tests.py` measured hover thrust from old queued messages and came out about 20% low, which made
  the drone dip during the angle steps. Fixed now.
