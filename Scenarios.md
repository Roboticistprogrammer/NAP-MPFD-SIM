Test scenarios

Since this is your initial data-collection phase, treat all of these as normal-condition baselines; fault-condition variants (mechanical/electrical/software/environmental, per your Ch.2/Ch.3 taxonomy) get layered in later as a second labeling pass on the same objects.

ID	Scenario	Description
SC-01	Static grasp-and-hold	Arm stationary, close on object, hold N seconds, release
SC-02	Full grasp-lift-hold-release cycle	Approach → grasp → lift → hold → lower → release
SC-03	Repeated-cycle endurance	Same object, N consecutive grasp cycles — useful later for degradation-trend data
SC-04	Variable approach orientation	Object rotated/offset between trials, same grasp target
SC-05	Graduated payload sweep	Same grasp motion across OBJ-A → OBJ-C in ascending weight order, single session
