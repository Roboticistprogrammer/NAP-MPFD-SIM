"""trial_logger.py tests — naming convention + metadata, subprocess mocked."""
import json
import os
from unittest.mock import MagicMock, patch

import pytest

from px4_gripper_missions.trial_logger import TrialLogger, bag_name, build_metadata


def test_bag_name_matches_05_ros2_data_logging_convention():
    assert bag_name('normal', 'SC02', 7) == 'normal_SC02_trial07'
    assert bag_name('fault_gps_stuck', 'SC01', 3) == 'fault_gps_stuck_SC01_trial03'


def test_bag_name_zero_pads_trial_number():
    assert bag_name('normal', 'SC01', 1) == 'normal_SC01_trial01'
    assert bag_name('normal', 'SC01', 42) == 'normal_SC01_trial42'


def test_build_metadata_matches_06_next_steps_schema():
    meta = build_metadata('normal', 'SC01', 1, 'OBJ-A', start_time_unix=1000.0)

    assert meta.trial_id == 'normal_SC01_trial01'
    assert meta.scenario_id == 'SC01'
    assert meta.object_id == 'OBJ-A'
    assert meta.condition_label == 'normal'
    assert meta.grasp_outcome == 'UNKNOWN'
    assert meta.data_source == 'sim'


def test_trial_logger_dry_run_does_not_touch_subprocess(tmp_path):
    with patch('subprocess.Popen') as popen:
        logger = TrialLogger(
            output_dir=str(tmp_path), condition_label='normal', scenario_id='SC01',
            trial_number=1, object_id='OBJ-A', dry_run=True,
        )
        logger.start()
        sidecar = logger.stop('SUCCESS')

        popen.assert_not_called()
        assert not os.path.exists(sidecar)  # dry-run never writes to disk


def test_trial_logger_writes_expected_bag_path_and_sidecar(tmp_path):
    fake_process = MagicMock()
    fake_process.wait.return_value = 0
    with patch('subprocess.Popen', return_value=fake_process) as popen:
        logger = TrialLogger(
            output_dir=str(tmp_path), condition_label='normal', scenario_id='SC02',
            trial_number=7, object_id='OBJ-B',
        )
        assert logger.bag_path == os.path.join(str(tmp_path), 'normal_SC02_trial07')

        logger.start()
        popen.assert_called_once()
        cmd = popen.call_args[0][0]
        assert cmd[:4] == ['ros2', 'bag', 'record', '-o']
        assert cmd[4] == logger.bag_path

        sidecar = logger.stop('SUCCESS')

    fake_process.send_signal.assert_called_once()
    assert sidecar == os.path.join(logger.bag_path, 'metadata_trial.json')
    with open(sidecar) as f:
        data = json.load(f)
    assert data['trial_id'] == 'normal_SC02_trial07'
    assert data['grasp_outcome'] == 'SUCCESS'
    assert data['data_source'] == 'sim'
    assert data['end_time_unix'] is not None


def test_trial_logger_context_manager_marks_aborted_on_exception(tmp_path):
    fake_process = MagicMock()
    fake_process.wait.return_value = 0
    with patch('subprocess.Popen', return_value=fake_process):
        with pytest.raises(RuntimeError):
            with TrialLogger(
                output_dir=str(tmp_path), condition_label='normal', scenario_id='SC01',
                trial_number=1, object_id='OBJ-A',
            ) as logger:
                raise RuntimeError('boom')

    with open(os.path.join(logger.bag_path, 'metadata_trial.json')) as f:
        data = json.load(f)
    assert data['grasp_outcome'] == 'ABORTED'


def test_trial_logger_stop_is_idempotent(tmp_path):
    fake_process = MagicMock()
    fake_process.wait.return_value = 0
    with patch('subprocess.Popen', return_value=fake_process):
        logger = TrialLogger(
            output_dir=str(tmp_path), condition_label='normal', scenario_id='SC01',
            trial_number=1, object_id='OBJ-A',
        )
        logger.start()
        first = logger.stop('SUCCESS')
        second = logger.stop('FAILURE')  # should be a no-op, not overwrite

    assert first == second
    with open(first) as f:
        data = json.load(f)
    assert data['grasp_outcome'] == 'SUCCESS'


def test_trial_logger_extra_metadata_mutation_reflected_in_sidecar(tmp_path):
    fake_process = MagicMock()
    fake_process.wait.return_value = 0
    with patch('subprocess.Popen', return_value=fake_process):
        logger = TrialLogger(
            output_dir=str(tmp_path), condition_label='normal', scenario_id='SC03',
            trial_number=1, object_id='OBJ-A',
        )
        logger.start()
        logger.extra_metadata['cycles'] = [{'cycle_index': 0, 'grasp_outcome': 'SUCCESS'}]
        sidecar = logger.stop('SUCCESS')

    with open(sidecar) as f:
        data = json.load(f)
    assert data['extra']['cycles'] == [{'cycle_index': 0, 'grasp_outcome': 'SUCCESS'}]
