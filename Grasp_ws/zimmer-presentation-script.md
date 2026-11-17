# Speaker script: Predicting grasp outcome from a short lift

Deck: `zimmer-presentation.pdf` (26 slides, November 2026). Target length: about 25 minutes plus discussion; the cut list at the end gives a 15-minute version. The October version is kept as `zimmer-presentation.oct17.pdf` with its script `zimmer-presentation-script.oct17.md`.
Written to be spoken, so the sentences are short. Change the wording to suit you.

---

## 1. Title (20 s)

Hi everyone. This is a progress update on the grasp-prediction work in the simulator. The short version: I took an idea from a soft-hand paper by Zimmer and colleagues and tried to make it work on a drone with a gripper. I'll show what I built and what I found. I'll also show a few things that don't work yet, because that is where I'd like your input.

## 2. Research question (45 s)

The question is simple. Can a drone's own flight data, recorded during a short, standard lift, tell us whether a grasp is going to hold?

The input is data the real drone already has: thrust, torque, motor commands, IMU, attitude and height. On top of that we only know whether the gripper is closed and where the arm was commanded to go.

There are two outputs. `y_success` says whether the object survives the full lift. `y_stable` says whether it also survives a sideways shake.

Why bother? Our real gripper has open-loop servos and no force feedback, so it can't tell when a grasp is bad. If we can predict failure early in the lift, the drone could abort and re-grasp before it drops the payload.

## 3. The short version (45 s)

If you only remember one slide, this is it. What works: the payload is clearly visible in the drone's own data, the 150-trial dataset has balanced labels and passed every acceptance gate, and once the window reaches one second the models detect a failing grasp with an AUC of about 0.91.

What does not work yet: early warning. At 0.4, 0.6 and 0.8 seconds the AUC is 0.51, 0.64 and 0.74, so we get only about 0.2 to 0.4 seconds of lead before the drop. The second label, `y_stable`, is not predictable from the small lift at all. And six ways of buying more lead time did not help. What decides what happens next is on the right: is detection enough, or do we need an input that senses the grasp itself? I'll end with questions.

## 4. Zimmer et al. 2019 (60 s)

This is the paper I'm adapting. They had a soft robotic hand on an arm. After closing the hand, they lifted the object a tiny bit, 10 mm over 1.5 seconds, and recorded the finger sensors. A classifier then predicted whether the full lift would succeed, with about 91 to 92 percent accuracy.

Two things matter for us. First, the small lift gave better predictions than the closing phase alone. Second, they labelled success with a scale and a wrist shake, which gave a clear ground truth.

The idea we borrow is: don't wait for the grasp to fail. Do a short standard lift and read the signals.

## 5. Mapping the protocol onto a UAV (75 s)

Here is how each piece translates. Their arm lifts 10 mm in 1.5 seconds. Our drone climbs 0.15 m in 1.5 seconds, and I'll explain the feed-forward part in a minute.

They had a USB scale. We don't need one, because the simulator tells us exactly how far the cylinder moved and how much it tilted, and that gives the labels.

They used sensors in the fingers. We use the drone's PX4 telemetry: 21 channels at 50 Hz, so each window is 75 samples.

Success for us means the cylinder is still held after a full one-metre lift and a four-second hold. Stability means it survives a 0.3 m sideways shake, but only if it was held in the first place.

The footnote matters: pressure or force sensing on the gripper stays an option, and I'll come back to it at the end.

## 6. The simulated testbed (75 s)

The setup runs in PX4 SITL with Gazebo Harmonic. The ground truth is fed to the estimator, so it behaves like a motion-capture system.

The drone is an x500 with our Armbot arm and a force-controlled two-finger gripper. The object is one small cylinder, 50 mm across, 120 mm long and 0.15 kg, sitting on a table.

There are two scenarios. In A the gripper grasps the cylinder at the centre, and it fails by sliding out. In B it grasps 30 mm off-centre, and it fails by pivoting around the fingers. So we get two different failure modes from the same object.

There's also an E scenario with an empty gripper. It's only a control and is not used for classification.

## 7. One trial, before and after (60 s)

This is the timeline of a single trial. The top row is the old protocol: descend, settle, close, lift 40 cm, hold.

The bottom row is the new one. Same start, but after closing there is the small lift, which is the 1.5-second window we want to predict from. Its start, t-zero, is the anchor for every window we cut. After that comes the full lift to one metre, a four-second hold, and then the shake.

The labels come at the end. `y_success` is decided after the hold and `y_stable` after the shake. The shake only runs if the cylinder is still in the gripper. A trial now takes about half a minute.

## 8. Labels (60 s)

How do we decide held versus failed? We use the simulator's ground truth: how far the cylinder moves relative to the grasp point.

More than 10 mm counts as slip onset. More than 40 mm means it's dropped, which is a failure. A tilt of more than 45 degrees also counts as failure.

`y_stable` uses the same rules, but counted from the start of the shake. If the grasp had already failed, `y_stable` is left empty, not set to zero.

We also log whether it failed by sliding or by tilting. Trials where the grasp never really happened, such as alignment timeouts, are thrown out.

## 9. What the model may see, and what it may not (60 s)

The left box is everything the model gets: thrust and torque setpoints, motor commands, body rates, accelerometer, roll and pitch, and the vertical state. That is 21 channels. It also gets two static facts: the gripper is closed and what the arm pose was commanded to be.

The right box is everything the model must not see, because it would be cheating. The big one is the grip force. In this simulator it largely decides the outcome, and the scenario, A or B, is basically a stand-in for it.

So to keep things honest, I report results per scenario against that scenario's own majority class. A model can't score well just by telling A from B.

## 10. Finding 1: the drone doesn't follow the commanded lift (75 s)

The first surprise. I commanded a smooth lift, and the drone barely moved. On the left, the dashed line is what I asked for and the solid line is what the drone did. At 1.5 seconds it had risen only 6 mm.

The reason is that the drone first has to take the cylinder's weight. Thrust goes up 3 to 4 percent with no motion for about 1.3 seconds. Then PX4's position loop adds about two more seconds of lag.

On the right, the fix: velocity feed-forward. Besides the position, we also send the velocity the profile needs. Now the drone follows about 85 percent of the lift. It's a standard PX4 offboard feature, so we can fly the same thing on the real drone.

## 11. Shorter distance or longer time? Neither. Feed-forward. (60 s)

I tried the obvious alternatives first. Each bar is how much of the commanded rise the drone actually climbed by the end of the window.

A shorter lift doesn't help. At 6 cm, the move is swallowed entirely: 0 percent. A longer lift helps only a little, 49 percent at 4 seconds, and a long window starts to contain the failures themselves.

Feed-forward is the only thing that works, giving 61 to 90 percent. I picked 0.15 m in 1.5 seconds with feed-forward. One caveat: each row is only 5 or 6 trials in scenario A, so this is a direction, not statistics.

## 12. Finding 2: held and failed grasps look alike until the drop (75 s)

This is the most important plot. It shows the thrust change during the window, averaged over held trials in blue and failed trials in orange.

Good news first. The payload is easy to see: thrust rises up to about 6 percent, and the hover noise is around 0.05 percent. So the signal is there.

The bad news is that the two curves are identical until about 0.9 seconds. The median drop happens at 0.98 seconds. They separate only around the drop, which means our window mostly contains the failure itself. Detecting a drop is easy. Warning before it is the hard part, and that is the real research question now.

## 13. Re-tuning the grip force (60 s)

Because the new lift is longer and higher, the old grip forces no longer worked. They held only 10 to 22 percent of grasps.

So I swept the force and fitted a curve. For A, the centre grasp, there is a sharp threshold at 1.46 newtons. For B, the off-centre grasp, the transition is shallower, with 50 percent at 2.75 newtons.

With those values, a 30-trial confirmation run gave 33 percent failures in each scenario, which is inside the 30 to 60 percent target. That's what we want: a balanced mix of outcomes.

## 14. Engineering fix: invalid trials (45 s)

A smaller fix, but it saved a lot of time. About a third of the A trials were invalid, meaning the gripper could not align on the cylinder.

The cause was the put-the-cylinder-back step after the shake. It was a one-metre descent that left the arm swinging and dropped the cylinder from about 15 cm. But the cylinder is teleported back to its jig before every trial anyway, so that step only cost time. I removed it. Invalid trials dropped to 3 out of 30, and six held-and-stable trials in a row had none.

## 15. Problem: y_stable was confounded with the scenario (45 s)

First a problem from last time. With one shared shake, scenario A was stable only 1 time in 10 and B 7 times in 8, among held grasps. A model could predict `y_stable` just by recognising A or B. The cause is the grip force: stability depends on it, and A and B sit at very different points on that curve at the forces that give 30 to 60 percent lift failures.	

## 16. Fix: one shake intensity per scenario (75 s)

The fix is a per-scenario shake. The ±0.3 metre amplitude and the grip forces stay; only the duration of each shake leg differs, and a longer leg is gentler. Each dot is the share of held grasps that fail the shake, for one setting; the dot size is the number of grasps.

For A, the 3.0-second legs of my first pilot were too gentle: none of 9 held grasps failed. At 2.0 seconds almost all did, 15 of 17. I interpolated and chose 2.25 seconds. For B, 1.25 seconds works. In the 150-trial batch, each scenario has 38 percent unstable among its held grasps. Each pilot only has about ten held grasps, so single points are noisy; the batch is what I trust.

## 17. Main batch: every acceptance gate passes (60 s)

The batch is 150 trials in four sessions, with the simulator restarted before each, because a long-running PX4 once drifted into an estimator fault. 135 trials are A or B, 131 are valid, which is 97 percent. Lift failures are 50 percent for A and 41 percent for B, inside the 30 to 60 percent target. 38 percent of held grasps are unstable in both scenarios, above the 25 percent gate. The 15 empty-gripper controls all came out empty, as they should. This gives 131 windows, 71 held and 60 failed.

## 18. Is the payload visible? Is anything leaking? (75 s)

Two checks from the plan. On the left, detectability: how big is the payload step compared with the hover noise? Thrust and motors are 200 times the noise over the whole window, and 26 to 48 times in the first 0.8 seconds. Pitch torque and vertical acceleration also clear the 3-times gate. Roll torque is weak in scenario A, probably because of geometry; I did not test that. One caution: the simulated hover is very quiet. Compared with the trial-to-trial scatter of the empty-gripper controls, thrust is 16 times over the window but only 2 to 4 times in the first 0.8 seconds. On the real drone the margin will be smaller.

On the right, leakage. No privileged variable is among the inputs. A model can tell A from B, with an AUC of 0.99, because the 30 millimetre offset shows in the torque. But the scenario alone says nothing about the labels: AUC 0.54 and 0.40. Grip force is readable only through the visible drop, not among held grasps. So the rules are: stratify the folds, report per scenario, never feed the force.

## 19. Baselines: the drop is detectable, early warning is not (90 s)

Here is the main result. On the left, when do the 60 failed grasps start to slip, in orange, and when do they fall, in teal? Slip starts at a median of 0.75 seconds, the drop follows at 1.06 seconds.

On the right is the AUC of an SVM and a small LSTM on the drone channels, with the window cut at the time on the axis, so a model cannot cheat by seeing the drop. At 0.4 seconds it is a coin flip, 0.51. At 0.6 it is 0.64, at 0.8 it is 0.74, and at 1.0 second it jumps to 0.91. The signal appears when the cylinder starts to slip, so the useful warning is about 0.2 to 0.4 seconds. The LSTM is no better than the SVM at 131 trials, A and B score alike, and holding out one whole session gives the same numbers, so there is no drift between simulator restarts. Finally, `y_stable` is not predictable from this window: AUC between 0.45 and 0.61 with 71 held grasps. With this size an AUC has an uncertainty of about 0.03 to 0.05.

## 20. Slower lift: the failure moves, the warning does not (75 s)

Then I tried to buy more lead time. First idea: a slower lift, 2.5 seconds instead of 1.5, so the failure should come later. It does. On the left, each dot is a failed grasp: slip start in orange and drop in teal, for the old lift in the bottom row and the slower lift in the top row. Both shift right by about half a second. The grip force for 50 percent held is almost unchanged: 1.49 newtons for A and 2.72 for B.

But on the right the AUC curve shifts by the same amount: chance up to 1.2 seconds, then 0.88 at 1.5 seconds. The model still needs the slip inside its window. This sweep has only 40 trials, so its AUC has an uncertainty of about 0.1; the pattern is what matters. Lower grip force is not a fix either: in scenario A the weakest grasps drop earliest, median 0.92 seconds against 1.10 for the strongest.

## 21. Four more attempts: nothing warns earlier (75 s)

The same idea four more ways. The gripper-closing window, which ends before the lift: AUC 0.49. Finger travel in the simulator: AUC below 0.5 up to 0.8 seconds, and it cannot even be measured on the open-loop servos. The commanded gripper state and the hover levels add nothing; the state is constant in every trial. And 33 more PX4 signals, such as tracking errors, controller outputs and the hover-thrust estimator: none beats the original 21 channels from 0.6 seconds on.

One small exception, a follow-up: the tracking errors at 0.4 seconds give an AUC of 0.62, with a p-value of 0.013. But I tested about 20 combinations, so one at that level is what chance produces. I treat it as a hypothesis for a fresh batch, not a finding. The conclusion at the bottom: with the drone's telemetry alone, the grasp margin is invisible until the cylinder starts to slip. More lead time needs an input that senses the grasp itself.

## 22. What the real gripper can and cannot give (60 s)

This is why I came to the real hardware. In the simulator the gripper is force-controlled and the finger position is exact. On the real Armbot the servos are commanded by angle, the squeeze comes from servo stiffness and stall torque, and nothing is fed back; the logger only records the commanded angle. I did not inspect the actual servo model, so please tell me if that is wrong.

So finger travel needs a hardware change, and the simulator says it would not help anyway. The inputs that could carry the margin before slip are proxies for the squeeze: servo current or load, a bus servo with load feedback, or a force pad. And I would not test that in simulation: a simulated force sensor would read the commanded force, which already predicts the outcome within scenario A with an AUC of 0.95. That is nearly circular. It needs hardware, or a sensor model with realistic noise and compliance.

## 23. Plan and status (30 s)

Status in one look. Baseline, payload detectability, the small-lift trajectory, the shake and labels, recording, the batch, and the dataset pipeline with the leakage check are all done. The baselines are partly done: SVM and LSTM, but no FFNN and no combined closing-plus-small-lift window. The CNN-LSTM and ablations are not justified at 131 trials, and the held-out forces and closed loop are still to do. 91 unit tests pass, and everything is on its own branch.

## 24. Ways forward (60 s)

Six options, with effort and my guess of how much they help with lead time. The first three are cheap: reframe the problem as detection, scored by detection delay against false alarms; a real-drone check of about 20 flights that only measures the payload step against the real hover noise; and a fresh batch to test the tracking-error hint, which will probably be a null. The sensing input is the only option with a plausible large gain, but it is hardware work. Matching the real knob with a position-controlled gripper, and changing the payload or pads for slower failures, have unknown effect. I propose doing the first three now and deciding the sensing input with you.

## 25. Questions for discussion (1/2) (open)

First, the contribution. Is detecting an impending drop about 0.2 to 0.4 seconds ahead enough, or must the thesis show earlier prediction? And what lead is useful on the real system, meaning how long do abort and re-grasp take? Second, the metric: should I move from AUC at fixed window ends to an online detector scored by detection delay against false alarms? Third, sensing: is a grasp-sensing channel in scope, and should it be modelled in the simulator first or go straight to hardware? Fourth, should I rebuild the simulated gripper as a position-controlled servo so the varied quantity matches the real one?

## 26. Questions for discussion (2/2) (open)

Fifth, real validation: how many real flights are needed to claim transfer, and is a first test with about 20 flights that only measures the payload step against the real hover noise a good start? Sixth, is a simulation-only result acceptable for this chapter if the real flights only confirm the signal level? Seventh, data: do I spend about 6 hours on a fresh held-out batch to test the one weak hint, or scale to 600 trials? Eighth, do we keep the CNN-LSTM, given that the LSTM equals the SVM today, and keep `y_stable` as a second task, or redefine it? Ninth, may I change the payload, friction or pads under our two-scenarios decision, if that gives slower, creep-like failures?

Thanks. Happy to take questions.

---

## Notes for the speaker

- **Likely questions.** "Why not just add a force sensor?" (slides 21 and 22: nothing in the drone data shows the margin before slip, and the simulator cannot test a sensor honestly.) "Isn't it too easy once the drop is in the window?" (slide 19: yes, 1.0 s and later is detection, not prediction; that is the honest framing.) "Does this transfer to the real drone?" (not yet; sim only, no rotor downwash, stiff arm joints, a much quieter hover.) "Is the tracking-error hint real?" (slide 21: p = 0.013 but about 20 tests, so a hypothesis, not a finding.)
- **Cut for a 15-minute slot:** skip slides 11, 13 and 14 (profile comparison, force tuning, invalid trials), mention slides 16 and 17 in one sentence each, and merge slides 25 and 26 into the first three questions.
- **Numbers to know.** 150 trials, 131 valid windows, 71 held, 60 failed, 44 stable and 27 unstable among the held. AUC (SVM, pooled) 0.51 / 0.64 / 0.74 / 0.91 / 0.95 at window ends 0.4 / 0.6 / 0.8 / 1.0 / 1.5 s. Slip starts at a median 0.75 s, drop at 1.06 s. Forces: A 1.46 N, B 2.75 N; shake legs A 2.25 s, B 1.25 s.
