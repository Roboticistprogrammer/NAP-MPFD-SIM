"""grasp_outcome.py's classify_outcome() — pure function, zero ROS dependency."""
from px4_gripper_missions.grasp_outcome import FAILURE, SUCCESS, UNKNOWN, classify_outcome


def test_no_baseline_is_unknown():
    assert classify_outcome([0.1, 0.1, 0.1], baseline_z=None, z_drop_threshold_m=0.02) == UNKNOWN


def test_no_readings_is_unknown():
    assert classify_outcome([], baseline_z=0.1, z_drop_threshold_m=0.02) == UNKNOWN


def test_readings_held_steady_is_success():
    readings = [0.100, 0.101, 0.099, 0.100]
    assert classify_outcome(readings, baseline_z=0.100, z_drop_threshold_m=0.02) == SUCCESS


def test_readings_within_threshold_noise_is_success():
    readings = [0.100, 0.095, 0.105, 0.090]  # within 0.02 of baseline
    assert classify_outcome(readings, baseline_z=0.100, z_drop_threshold_m=0.02) == SUCCESS


def test_object_dropped_is_failure():
    readings = [0.100, 0.099, 0.05, 0.02]  # falls well past the threshold
    assert classify_outcome(readings, baseline_z=0.100, z_drop_threshold_m=0.02) == FAILURE


def test_single_dip_below_threshold_anywhere_is_failure():
    # Even a brief dip counts — classify_outcome checks every reading,
    # not just the final one.
    readings = [0.100, 0.070, 0.100, 0.100]
    assert classify_outcome(readings, baseline_z=0.100, z_drop_threshold_m=0.02) == FAILURE


def test_reading_exactly_at_threshold_boundary_is_success():
    readings = [0.100 - 0.02]  # exactly baseline - threshold, not below it
    assert classify_outcome(readings, baseline_z=0.100, z_drop_threshold_m=0.02) == SUCCESS
