# 05 — Recording Simulation Data

## 1. Topics to record

Same module set as the real-vehicle fusion plan — recording these in
sim keeps the schema identical to what you'll log in flight:

```
/fmu/out/sensor_combined
/fmu/out/vehicle_attitude
/fmu/out/vehicle_local_position
/fmu/out/vehicle_gps_position
/fmu/out/battery_status
/fmu/out/actuator_motors
/fmu/out/estimator_status
/fmu/out/vehicle_imu_status
/fmu/out/rc_channels
```

(`esc_status` is not meaningful in SITL since there's no real ESC
telemetry to simulate — omit it here, but keep it in your real-vehicle
recording list.)

## 2. Record with rosbag2

```bash
mkdir -p ~/sim_data
cd ~/sim_data
ros2 bag record -o run_$(date +%Y%m%d_%H%M%S) \
  /fmu/out/sensor_combined \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_gps_position \
  /fmu/out/battery_status \
  /fmu/out/actuator_motors \
  /fmu/out/estimator_status \
  /fmu/out/vehicle_imu_status \
  /fmu/out/rc_channels
```

Start this **after** the SITL sim and XRCE-DDS agent are both running
and topics are confirmed live (`ros2 topic list`). Stop with Ctrl+C when
the trial/scenario is complete — one bag per trial, same logic as the
per-trial CSV files from the ArmBot logger.

## 3. Tag each recording

Name the output directory (via `-o`) to encode the same metadata
convention used for the gripper logs:

```
{condition_label}_{scenario_id}_trial{NN}
```

e.g. `normal_SC02_trial07`, `fault_gps_stuck_SC01_trial03`.

## 4. Convert to a format usable by the fusion pipeline

Rosbag2 (SQLite3 or MCAP) isn't directly consumable by
`pandas`/`filterpy`. Use the `rosbags` Python library (no ROS
installation needed on the processing side) to convert to CSV/parquet:

```bash
pip install rosbags
```

```python
from rosbags.highlevel import AnyReader
from pathlib import Path
import pandas as pd

def bag_to_dataframe(bag_path: str, topic: str) -> pd.DataFrame:
    rows = []
    with AnyReader([Path(bag_path)]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            row = {"host_timestamp_ns": timestamp}
            row.update(vars(msg))  # adjust per message type as needed
            rows.append(row)
    return pd.DataFrame(rows)
```

Run this per topic per bag, then merge with the gripper CSV log using
`pandas.merge_asof` on timestamp, as already planned for the
sync/fusion stage.

## Checklist before moving on

- [ ] A test recording completes and produces a non-empty bag directory
- [ ] Bag can be read back with `ros2 bag info <path>`
- [ ] `rosbags`-based conversion produces a DataFrame with expected columns
- [ ] Naming convention matches the ArmBot logger's convention for easy pairing
