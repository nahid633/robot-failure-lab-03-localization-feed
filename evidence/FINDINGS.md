# Robot Failure Lab 03: the same stop, a different task outcome

Measured October 18, 2026. Standalone Python runner, ROS 2 Jazzy, Nav2 and a simulated TurtleBot3 in Gazebo. Nine planned main trials; three separately retained verification trials.

## Observed result

| Condition | Original goals completed | Stops during outage window | Resumed after a measured stop |
| --- | --- | --- | --- |
| Normal | 3/3 | 0/3 | 0/3 |
| Full outage | 0/3 | 3/3 | 0/3 |
| Localization kept fed | 3/3 | 3/3 | 3/3 |

## Interpretation

Keeping fresh scans flowing to AMCL localization changed the observed task outcome while the other branch still lost scans for two seconds. The full outage reproduced transform error 102 in all three main trials. With localization kept fed, all three robots stopped, resumed, and completed the original goal without a replacement goal.

This supports the hypothesis that loss of AMCL scan input contributes to the transform-related goal abort in this configuration. It also demonstrates that a sensor-related stop can be followed by continuation of the original task in the modified routing condition. It does not establish that each consumer’s causal contribution has been isolated, or that forwarding fresh localization scans is a production remedy for a physically failed scanner.

The collision monitor stayed enabled with its original one-second source timeout. AMCL settings, transform tolerances, both costmaps and the navigation behavior were unchanged. Both branches used the same relay topology in all three conditions; only their gate policy changed.

## Timing and transform evidence

- Full-outage stop latency: 1.044–1.086 simulated seconds after onset.
- Localization-fed stop latency: 1.050–1.086 simulated seconds after onset.
- Localization-fed resumption: 0.166–0.217 simulated seconds after the window ended.
- Each outage branch discarded ten scans in each relevant trial; the localization branch stayed continuous in the localization-fed condition. Forwarding returned after every outage. Independent observer receipts verify branch delivery, not use inside every consumer.
- Map/odom lookup probes recorded an unavailable interval in each full-outage run and no unavailable samples in the tested normal/localization-fed windows and four-second tails. Scan restoration and transform availability returned after the full outage, but the original goal was already terminal. See EVIDENCE-VALIDATION.json and the raw transform CSVs for counts/times.
- Exact-current-time map/base probes also frequently missed during successful normal trials, so that probe did not discriminate the conditions. These diagnostic misses are not additional navigation failures. The map/odom probe and actual controller errors provide the relevant contrast.
- Task times still vary. They do not establish a causal speed improvement. Failed-goal termination times are not completed journey times. Trigger positions are odometry coordinates, not independently measured map positions.

## Every main trial

| Block | Condition | Outcome | Goal-to-termination (sim s) | Error | Stop (s after onset) | Resume (s after end) | Recoveries |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Normal | succeeded | 15.150 | 0 | — | — | 0 |
| 1 | Full outage | failed | 4.065 | 102 | 1.086 | — | 0 |
| 1 | Localization kept fed | succeeded | 30.129 | 0 | 1.050 | 0.166 | 3 |
| 2 | Full outage | failed | 4.050 | 102 | 1.086 | — | 0 |
| 2 | Localization kept fed | succeeded | 56.469 | 0 | 1.065 | 0.217 | 5 |
| 2 | Normal | succeeded | 15.255 | 0 | — | — | 0 |
| 3 | Localization kept fed | succeeded | 16.164 | 0 | 1.086 | 0.202 | 0 |
| 3 | Normal | succeeded | 17.904 | 0 | — | — | 0 |
| 3 | Full outage | failed | 4.056 | 102 | 1.044 | — | 0 |

## All attempts and teardown anomalies

The three excluded verification trials all passed and remain under excluded-checks/verification. Their results are not included in the nine-trial counts.

1 attempted main launch(es) timed out before readiness, with no goal or outage submitted. Logs remain in the original attempt directory; its successful retry has a distinct attempt suffix. Continuation preserved the three already completed outcomes, used a fresh ROS domain (191 initially, then 192), and kept all measurement/worker code and parameters frozen.

Collision-monitor exit code -11 was recorded during teardown in 2 main trial(s): block-1-localization-fed, block-1-normal. These events followed the runner’s shutdown signal after the recorded task result and observation period. They are retained anomalies, not task aborts during the measured outage. Other nonzero exits from the intentional SIGINT/forced shutdown are also preserved in ATTEMPT-AUDIT.json. Their underlying causes were not diagnosed.

## Checks and limits

Six focused logic/routing/measurement tests passed. All nine main records passed gate-by-gate delivery, independent branch receipt, upstream continuity, scan-age, routing, odometry-sampling, goal-state and recomputed-metric checks. Frozen source hashes were rechecked. Installed package snapshots and all navigation parameters except scan routing matched Lab02. Lab02’s existing 169 checksum entries were verified unchanged.

Three repeats per condition, one static simulated route. Gazebo seeds 101/102/103 were matched within blocks; AMCL randomness, scheduling, exact paths and trigger positions remain uncontrolled. This is not a statistical failure-rate estimate or general Nav2 reliability claim. The costmaps and collision monitor remain grouped, and timing mediation has not been fully isolated. Odometry is simulated feedback. No physical safety, new-obstacle collision performance, Vektor integration/performance, adoption, customer demand, publication or outbound message is demonstrated. Lab01’s delay effect remains unresolved; Lab01b remains unexecuted.

## Retained evidence

REPORT.md contains the measurement and diagnostic tables. comparison.png shows all nine outcomes; first-block-timeline.png shows the predeclared first block. PROTOCOL.json, source/, params.yaml, packages.txt, every trial’s logs/CSVs/VALIDATION.json, ATTEMPT-AUDIT.json, continuation records, excluded checks and SHA256SUMS.txt retain provenance. reproduction/ supplies the unchanged code with the minimal Lab02 reference files it needs; a fresh-machine reproduction has not been verified.

## Selected full-outage log evidence

### block-1-full-outage

```text
[collision_monitor-17] [WARN] [1792352072.217646298] [collision_monitor]: Robot to stop due to invalid source. Either due to data not published yet, or to lack of new data received within the sensor timeout, or if impossible to transform data to base frame
[controller_server-9] [ERROR] [1792352072.339923024] [controller_server]: Exception in transformPose: Lookup would require extrapolation into the future.  Requested time 20.448000 but the latest data is at time 20.401000, when looking up transform from frame [map] to frame [odom]
[controller_server-9] [ERROR] [1792352072.340033813] [controller_server]: Unable to transform goal pose into costmap frame
[bt_navigator-14] [ERROR] [1792352072.367602237] [bt_navigator]: Goal failed
```

### block-2-full-outage-attempt-2

```text
[collision_monitor-17] [WARN] [1792352254.981775107] [collision_monitor]: Robot to stop due to invalid source. Either due to data not published yet, or to lack of new data received within the sensor timeout, or if impossible to transform data to base frame
[controller_server-9] [ERROR] [1792352255.094042906] [controller_server]: Exception in transformPose: Lookup would require extrapolation into the future.  Requested time 20.448000 but the latest data is at time 20.401000, when looking up transform from frame [map] to frame [odom]
[controller_server-9] [ERROR] [1792352255.094154998] [controller_server]: Unable to transform goal pose into costmap frame
[bt_navigator-14] [ERROR] [1792352255.120308518] [bt_navigator]: Goal failed
```

### block-3-full-outage

```text
[collision_monitor-17] [WARN] [1792352463.557355711] [collision_monitor]: Robot to stop due to invalid source. Either due to data not published yet, or to lack of new data received within the sensor timeout, or if impossible to transform data to base frame
[controller_server-9] [ERROR] [1792352463.705189505] [controller_server]: Exception in transformPose: Lookup would require extrapolation into the future.  Requested time 20.448000 but the latest data is at time 20.401000, when looking up transform from frame [map] to frame [odom]
[controller_server-9] [ERROR] [1792352463.705315441] [controller_server]: Unable to transform goal pose into costmap frame
[bt_navigator-14] [ERROR] [1792352463.730869830] [bt_navigator]: Goal failed
```

