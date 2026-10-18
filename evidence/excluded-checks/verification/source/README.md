# Robot Failure Lab 03

Standalone ROS 2 Jazzy / Gazebo / Nav2 TurtleBot3 simulation. Two scan outputs use
the same subscriber topology in every condition. Compare normal, full two-second
outage, and an outage to both costmaps/collision monitor with AMCL still fed.
Never disables the collision monitor or changes transform timeouts. No new goal
is sent after an abort. All task outcomes remain part of the evidence.

Supported runtime: Ubuntu/WSL with ROS 2 Jazzy, Nav2 bringup and Gazebo installed.
From this directory:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest -v test_core
python3 run.py /tmp/rfl03-verification --verification --domain 191
python3 run.py /tmp/rfl03-main --verification-root /tmp/rfl03-verification --domain 191
python3 report.py /tmp/rfl03-main
```

Use new output directories and an idle ROS domain. A snapshot is executed and
hashed before each series. Main runs require all three excluded verification
conditions to pass evidence checks using the same code/parameters. Main order:
seed101 normal/full/localization-fed; seed102 full/localization-fed/normal;
seed103 localization-fed/normal/full. AMCL randomness and scheduling uncontrolled.

Gate: after >=3 simulated seconds of goal execution and sustained translation
>=0.05m/s for >=0.3s with odometry age <=0.15s; drop for [trigger, trigger+2s).
Preserve acquisition timestamps; no buffering or replay. Task bound 90 simulated
or 270 wall seconds; startup bound 60 wall seconds after worker launch.
Record passively through >=4 simulated seconds after the outage, even after task
abort. A bounded tail watchdog protects against a stalled clock.

The independent branch subscriptions validate observer receipt, not processing
inside every Nav2 consumer. Zero-timeout transform probes test map/odom and full
map/base lookups at the current simulated time; exact-time full-chain probes may
miss between odometry callbacks. Future-dated map/odom headers are valid-until
times, not localization accuracy. Keep task errors distinct from diagnostic misses.

On infrastructure failure, inspect retained logs. --resume-infrastructure skips
validated outcomes and only continues before-goal startup failures, retaining all
attempts and logging continuation. Never use it to retry a task result or change
measurement code mid-series. Maximum three startup attempts per scheduled trial.
Protocol/evidence errors stop the batch. No automatic result-based extensions.

Report all attempts and failures; do not pool historical Lab02 trials with this
sample. This is a small static simulation, not a general reliability estimate,
physical safety test, collision benchmark, or evaluation/integration of Vektor.
