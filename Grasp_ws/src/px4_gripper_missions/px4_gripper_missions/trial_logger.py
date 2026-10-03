"""ros2 bag record wrapper + sidecar trial metadata.

Naming convention and topic list are locked by
05_ros2_data_logging.md — this module is the single place that
convention is encoded, so no scenario script duplicates it.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional

# Exact list from 05_ros2_data_logging.md's `ros2 bag record` command.
BAG_TOPICS = [
    '/fmu/out/sensor_combined',
    '/fmu/out/vehicle_attitude',
    '/fmu/out/vehicle_local_position',
    '/fmu/out/vehicle_gps_position',
    '/fmu/out/battery_status',
    '/fmu/out/actuator_motors',
    '/fmu/out/estimator_status',
    '/fmu/out/vehicle_imu_status',
    '/fmu/out/rc_channels',
]

DATA_SOURCE = 'sim'  # per 06_next_steps.md's real-vs-sim column


def bag_name(condition_label: str, scenario_id: str, trial_number: int) -> str:
    """Exact naming convention from 05_ros2_data_logging.md:
    {condition_label}_{scenario_id}_trial{NN}, e.g. normal_SC02_trial07."""
    return f'{condition_label}_{scenario_id}_trial{trial_number:02d}'


@dataclass
class TrialMetadata:
    trial_id: str
    scenario_id: str
    object_id: str
    condition_label: str
    grasp_outcome: str
    data_source: str
    start_time_unix: float
    end_time_unix: Optional[float] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)


def build_metadata(
    condition_label: str,
    scenario_id: str,
    trial_number: int,
    object_id: str,
    start_time_unix: float,
    extra: Optional[Dict[str, Any]] = None,
) -> TrialMetadata:
    return TrialMetadata(
        trial_id=bag_name(condition_label, scenario_id, trial_number),
        scenario_id=scenario_id,
        object_id=object_id,
        condition_label=condition_label,
        grasp_outcome='UNKNOWN',
        data_source=DATA_SOURCE,
        start_time_unix=start_time_unix,
        # `extra or {}` would silently swap in a *new* dict whenever the
        # caller's extra_metadata happens to still be empty (falsy) at
        # start() time — breaking TrialLogger's public contract that
        # mutating .extra_metadata after start() is reflected at stop().
        # Explicit None-check preserves the reference regardless of
        # truthiness (caught by test_trial_logger_extra_metadata_mutation_reflected_in_sidecar).
        extra=extra if extra is not None else {},
    )


class TrialLogger:
    """Wraps `ros2 bag record` for one trial + writes a sidecar
    metadata_trial.json matching 06_next_steps.md's fused-CSV column
    schema (trial_id/scenario_id/object_id/condition_label/
    grasp_outcome/data_source).

    Supports the context-manager protocol: if the `with` block exits
    via an exception before stop() was called with a real outcome,
    grasp_outcome is recorded as "ABORTED" rather than being silently
    left UNKNOWN.
    """

    def __init__(
        self,
        output_dir: str,
        condition_label: str,
        scenario_id: str,
        trial_number: int,
        object_id: str,
        extra_metadata: Optional[Dict[str, Any]] = None,
        dry_run: bool = False,
    ):
        self._output_dir = output_dir
        self._condition_label = condition_label
        self._scenario_id = scenario_id
        self._trial_number = trial_number
        self._object_id = object_id
        # Public and mutable on purpose: a scenario script (e.g. SC-03's
        # per-cycle outcomes) can add to this dict any time before
        # stop() is called, and it'll be reflected in the sidecar JSON
        # — build_metadata() stores this same dict by reference, not a
        # copy, and serialization only happens inside stop().
        self.extra_metadata: Dict[str, Any] = extra_metadata or {}
        self._dry_run = dry_run

        self._name = bag_name(condition_label, scenario_id, trial_number)
        self._bag_path = os.path.join(output_dir, self._name)
        self._process: Optional[subprocess.Popen] = None
        self._metadata: Optional[TrialMetadata] = None
        self._stopped = False

    @property
    def bag_path(self) -> str:
        return self._bag_path

    def start(self) -> None:
        os.makedirs(self._output_dir, exist_ok=True)
        self._metadata = build_metadata(
            self._condition_label,
            self._scenario_id,
            self._trial_number,
            self._object_id,
            start_time_unix=time.time(),
            extra=self.extra_metadata,
        )

        if self._dry_run:
            print(f'[dry-run] would start: ros2 bag record -o {self._bag_path} '
                  f'{" ".join(BAG_TOPICS)}')
            return

        self._process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-o', self._bag_path, *BAG_TOPICS],
        )

    def stop(self, grasp_outcome: str) -> str:
        """Stops recording, writes the sidecar JSON, returns its path."""
        if self._stopped:
            return self._sidecar_path()
        self._stopped = True

        if not self._dry_run and self._process is not None:
            self._process.send_signal(signal.SIGINT)
            try:
                self._process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()

        if self._metadata is None:
            self._metadata = build_metadata(
                self._condition_label, self._scenario_id, self._trial_number,
                self._object_id, start_time_unix=time.time(), extra=self.extra_metadata,
            )
        self._metadata.grasp_outcome = grasp_outcome
        self._metadata.end_time_unix = time.time()

        sidecar_path = self._sidecar_path()
        if self._dry_run:
            print(f'[dry-run] would write {sidecar_path}:\n{self._metadata.to_json()}')
        else:
            os.makedirs(self._bag_path, exist_ok=True)
            with open(sidecar_path, 'w') as f:
                f.write(self._metadata.to_json())

        return sidecar_path

    def _sidecar_path(self) -> str:
        return os.path.join(self._bag_path, 'metadata_trial.json')

    def __enter__(self) -> 'TrialLogger':
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if not self._stopped:
            self.stop('ABORTED' if exc_type is not None else 'UNKNOWN')
