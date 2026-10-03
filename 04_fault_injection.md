# 04 — Fault Injection in SITL

This is the part of the pipeline that generates the "unsafe fault
class" data — failures too risky to induce on the real UAV in flight.

## Important caveat — verify support before relying on this

PX4's `failure` command is documented as having **variable support
across simulators and failure types** — at time of the underlying PX4
docs, Gazebo Classic was the confirmed-supported simulator, and support
in the newer Gazebo (Harmonic/gz-sim) may be partial. Some failure
types will return an `unsupported` message. Treat this file as a
starting point to be validated against your specific PX4 version, not
a guaranteed-working list.

**Action item for your agent:** run each failure type below once, log
whether it returns `unsupported` or actually changes vehicle behavior,
and keep only the confirmed-working subset as your fault-injection
method for that class. Document this validation table in the thesis —
it's a legitimate methodological transparency point, not just a
debugging note.

## 1. Enable failure injection

In the PX4 shell (`pxh>`) or QGC's MAVLink Console:

```
param set SYS_FAILURE_EN 1
```

## 2. Command syntax

```
failure <target> <type> [instance]
```

- `target`: the subsystem to fail (e.g. `gps`, `rc_signal`, `gyro`, `accel`, `mag`, `baro`, `motor`)
- `type`: failure mode (e.g. `off`, `stuck`, `wrong`, `ok` to restore)
- `instance`: optional, which sensor instance (0 = all)

## 3. Examples to validate first

```
# GPS
failure gps off      # dead receiver, no position
failure gps stuck     # frozen last-known fix
failure gps wrong     # diverging/incorrect position
failure gps ok        # restore

# RC link
failure rc_signal off
failure rc_signal ok
```

Sensor targets worth testing against your project's fault taxonomy
(mechanical/electrical/software/environmental) — check which of `gyro`,
`accel`, `mag`, `baro` respond in your Gazebo version, since these map
most directly to your "sensor fault" category.

## 4. Actuator/motor degradation — alternative method

The native `failure motor ...` target is one of the less consistently
implemented types. A more reliable way to simulate partial
motor/actuator degradation in SITL is direct actuator override:

```
actuator_test set -m <motor_number> -v <value> -t <timeout_seconds>
```

`-v` ranges -1 to 1 (commanded output level) — setting a reduced value
on one motor while others run normally approximates partial
thrust-loss, a common real-world actuator degradation mode. This is a
manual override tool, not a subtlety-graded degradation model, so use
it for discrete fault-state generation (e.g., "motor 2 at 60% output")
rather than continuous degradation trends.

## 5. Recommended fault set for this project

Given the caveats above, plan for:

| Fault class | SITL method | Confirm before relying on |
|---|---|---|
| GPS degradation | `failure gps stuck/off/wrong` | Usually well-supported |
| RC link loss | `failure rc_signal off` | Usually well-supported |
| IMU/sensor fault | `failure gyro/accel/mag/baro off\|stuck` | Test each — support varies |
| Partial motor failure | `actuator_test set -m <n> -v <reduced>` | Manual method, reliable |
| Grasp-induced disturbance | `ApplyLinkWrench` (see file 03) | Custom, not a PX4 native fault |

## Checklist before moving on

- [ ] `SYS_FAILURE_EN` set to 1 and persists for the session
- [ ] Each planned fault type tested once and outcome logged (supported / unsupported)
- [ ] At least one confirmed working example per fault class in your taxonomy
- [ ] `actuator_test` validated as the fallback for actuator degradation
