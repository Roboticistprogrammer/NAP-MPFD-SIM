# 06 — How This Feeds the Fusion Pipeline

## Scope reminder

This simulation stage does not change any decisions already made:

- **One fusion method** (EKF via `filterpy`) — sim data is additional
  *training/validation data* for that one pipeline, not a reason to
  build a separate one.
- **Physical fault injection on the real ArmBot remains primary.**
  Gazebo/PX4 fills two specific gaps only: (1) pre-flight software
  validation, (2) unsafe fault classes that can't be induced on real
  hardware in flight.
- **`scenario_id` / `condition_label` tagging** — simulated trials get
  tagged exactly like real trials (see `05_ros2_data_logging.md`
  naming convention), so they merge into the same dataset rather than
  forming a separate track.

## Mapping to your five graduated test cases

Run the relevant fault-injection and disturbance scripts (files 03–04)
*inside* each of your existing five flight scenarios (static hover →
autonomous inspection), the same way it was decided for the physical
gripper scenarios — not as a separate scenario set.

## Immediate next steps

1. Complete files 01–02 (environment + baseline sim) and confirm the checklist items.
2. Validate the fault-injection table in `04_fault_injection.md` — this
   determines which fault classes you can actually source from sim.
3. Record a handful of normal-condition sim trials and confirm they
   merge cleanly with real ArmBot logs via `merge_asof`, using the same
   host-timestamp sync approach already used for the real vehicle.
4. Feed both real and simulated trials into the trial-based
   train/val/test split — keep sim-origin trials distinguishable (e.g.
   a `data_source` column: `real` vs `sim`) so you can later report
   sim-to-real performance gap as part of your validation, not hide it.
   
   
## Questions:
so i will get fused csv file for each scenario and i will train algorithm on it for prediction? if this is right give info on the training proceedure tech stack

Clarified individual trial file pooling and windowing methodology
Clarified individual trial file pooling and windowing methodology
Correction on the mental model

Not quite one CSV per scenario — it's one fused CSV per trial (each grasp attempt / flight run), tagged with trial_id, scenario_id, object_id, condition_label, and now grasp_outcome. Many trials across all your scenarios/objects/conditions aggregate into one dataset. The model doesn't train on a single scenario file — it trains on windows sampled across all of them.

Step 1: Windowing (the missing piece)

CNN-LSTM doesn't consume raw per-timestamp rows directly — it consumes fixed-length sequences.

Slide a window (e.g. 2–5 seconds at your fused sample rate) over each trial's fused CSV
Each window becomes one training sample: (sequence_of_features, label)
Labels per window: fault class, grasp_outcome, health score, alert level — all four heads pull from the same window
Never window across a trial boundary — a window must come from a single trial, or you leak unrelated dynamics together
Step 2: Trial-based split (already decided, applies here directly)

Split by trial_id — not by window — into train/val/test. All windows from one trial stay in the same split. This is the temporal-leakage rule already established; windowing makes it easy to violate accidentally if you're not careful, so enforce it at the trial level before windowing.

Step 3: Training procedure
Step	What happens
1. Load	pandas reads all trial CSVs, tags each row with metadata
2. Window	Slide fixed-length windows per trial, build (X, y) pairs
3. Split	Trial-based train/val/test (e.g. 70/15/15)
4. Normalize	Fit scaler on train only, apply to val/test
5. Dataset/DataLoader	torch.utils.data.Dataset wraps windows, DataLoader batches
6. Forward pass	CNN extracts spatial/local patterns per window → LSTM models temporal trend → shared representation feeds 4 heads
7. Loss	Weighted sum: cross-entropy (fault class), cross-entropy (grasp_outcome), MSE (health score / RUL), cross-entropy (alert level)
8. Optimize	Adam/AdamW, LR scheduler (cosine or step decay)
9. Validate each epoch	Track per-head metrics separately, not just combined loss
10. Early stopping	On validation combined loss or grasp_outcome F1, whichever is your primary success criterion
11. Test	Final held-out trials, report per-head metrics + ablations
Tech stack (training-specific additions to what's already set)
Component	Tool
Windowing/dataset prep	numpy, pandas
Model + training loop	PyTorch (torch.nn, custom multi-task nn.Module)
Multi-task loss weighting	Manual weighted sum, or uncertainty weighting (learnable log-variance per task) if one head dominates
Experiment tracking	MLflow / Weights & Biases — log per-head metrics per epoch
Metrics	torchmetrics (F1, AUROC), scikit-learn (confusion matrix)
Checkpointing	torch.save best-val checkpoint per run
Cross-validation	Group K-fold by trial_id (sklearn.model_selection.GroupKFold) for robustness beyond a single split
