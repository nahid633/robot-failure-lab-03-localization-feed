# Robot Failure Lab 03: does keeping localization fed change blackout recovery?

## Recommendation and status

Planning proposal, awaiting the founder's choice. No implementation or simulation is authorized by the current request. Recommend investigating one mechanism behind Lab 02 before introducing another fault. The bounded question is: during the same two-second outage to obstacle sensing and the collision monitor, does keeping localization supplied with fresh scans prevent the transform-related goal abort and allow the original task to continue?

Localization estimates where the robot is on the map. AMCL is the localization component used here. Costmaps represent obstacles for planning and local motion. The collision monitor can limit motion when its sensor input is stale. A coordinate transform connects position descriptions, such as the map and wheel-motion reference frames. These roles can fail differently even though they share one laser scanner.

## Evidence and alternatives

Observed in Lab 02: normal 3/3 successes; blackout 0/3 successes; stops after 1.086–1.176 simulated seconds; scan forwarding returned in every blackout trial, but no motion resumed through at least four seconds after the window. All three blackout actions returned error 102 (TF_ERROR), with stale-source warnings and map/odom transform errors. No replacement goal was sent. The relay supplied AMCL, both costmaps and the collision monitor, so their individual contributions were not isolated. Sources: [robot-failure-lab-02-scan-blackout](<C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/wiki/experiments/robot-failure-lab-02-scan-blackout.md>), [archived findings](C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/artifacts/robot-failure-lab-02/FINDINGS.md), and [measured report](C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/artifacts/robot-failure-lab-02/REPORT.md).

Mechanism follow-up: change which consumers lose the existing scan stream, then inspect task and transform behavior. This makes a specific explanation testable. A new-fault experiment, such as intermittent scan loss rather than one continuous outage, would instead explore another operating condition; it would leave the present mechanism question open. A duration sweep would test an outage threshold, also a different question. Recommendation: choose this mechanism follow-up first. Neither alternative is selected or scheduled.

Lab 01's six successful runs did not establish a reliable 500 ms delay effect because timing varied widely with uncontrolled randomness. [robot-failure-lab-01b-repeatability](<C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/wiki/experiments/robot-failure-lab-01b-repeatability.md>) remains proposed and unexecuted; this proposal does not resolve that uncertainty.

## Proposed conditions

Use the same two scan branches and subscriber assignments in every condition. Both branches originate from the same observer relay and preserve scan payloads, acquisition timestamps and sensor-data QoS. Only the gate policy changes; AMCL never gets a special direct connection to the raw scanner in one condition.

| Condition | AMCL localization branch | Both costmaps and collision monitor branch | Purpose |
| --- | --- | --- | --- |
| A: normal | Forward continuously | Forward continuously | Check normal operation with the new branch topology |
| B: full outage | Drop new scans for two seconds | Drop new scans for the same two seconds | Reproduce Lab 02 behavior in the new runner |
| C: localization kept fed | Forward continuously | Drop new scans for two seconds | Test the effect of preserving AMCL input during the other consumers' outage |

B versus C is the primary comparison. Keep all collision-monitor settings enabled and unchanged, including the installed one-second source timeout. Do not extend transform tolerances, publish substitute transforms, fabricate scans, reset localization or send replacement goals. This is a diagnostic routing intervention, not a proposed repair for a physically failed laser scanner.

## Bounded protocol, only after selection

1. Implement in a separate Experiment 03 directory. Preserve Lab 01 and the complete Lab 02 archive, source snapshots, excluded checks, startup failures, continuation history and user-facing outputs. Reuse inspected measurement logic where suitable; the current Lab 02 runner has one output and checks that all four consumers share it, so split routing is new work, not an existing capability.
2. Verify installed versions and parameters against the Lab 02 snapshot. Record any differences before freezing the protocol. Keep ROS 2 Jazzy, Gazebo, TurtleBot3, the same static map, start (-2, -0.5), goal (1, 0.5), yaw zero, navigation configuration and machine conditions fixed. Fresh simulator and Nav2 for each trial, equal 1.5 simulated-second wait after readiness, no concurrent experiment workload.
3. Check branch gate boundaries with focused tests. After selection, allow up to three separately labeled verification trials, one per condition, to check routing and complete observation; retain them outside the main sample. If verification fails, retain evidence and revise the protocol before starting the main batch. Do not silently mix development trials with main results.
4. Before every goal, save a live subscriber graph: raw scans feed only the observer; the localization branch feeds AMCL; the other branch feeds both costmaps and the collision monitor. Compare effective configurations across conditions, allowing only the declared gate policy to differ. Verify upstream and both branch streams, not just topic names. Forwarding does not prove every consumer processed every scan.
5. Main budget: nine trials, three blocks with Gazebo seeds 101, 102 and 103. Predeclare block orders A/B/C, B/C/A, C/A/B. Match the Gazebo seed within each block; record supported AMCL seed controls if verified before protocol freeze, otherwise retain AMCL randomness as uncontrolled. Matching Gazebo seeds does not make trajectories deterministic. Record host load, real/simulated elapsed times and trigger position.
6. Trigger the common two-second window after at least three simulated seconds since goal submission and translation >=0.05 m/s continuously for >=0.3 seconds, using odometry no older than 0.15 seconds. Mark an equivalent window in A. Drop newly arriving scans during [trigger, trigger+2 s), with no buffer, timestamp rewriting or replay. Restore forwarding immediately afterward.
7. Retain the 90 simulated-second task deadline, 270-wall-second task bound and 60-wall-second startup-readiness bound. Continue passive recording through at least four simulated seconds after the window even if the action aborts, using a bounded tail watchdog. If a task remains active, observe until success, failure or the declared task limit. Keep task termination and observation end separate. No replacement goal is sent.
8. Retain all attempted trial IDs. Startup errors before a goal are infrastructure errors; task failures and timeouts remain outcomes. Missing triggers, incorrect routing or incomplete outage evidence are protocol failures, explicitly reported rather than counted as successful tests. Stop the batch to inspect infrastructure/protocol failures; do not rerun completed task outcomes or extend the sample after seeing results. Any continuation must preserve the original attempt and be documented.

This is an exploratory budget, not a statistical power calculation. Repeated seeds are not distinct environments.

## Measurements

Primary: original navigation-goal state and terminal result/error, including whether the goal remains active through the outage and ultimately succeeds. Report every trial and each condition's counts. Scan restoration, motion resumption and goal completion are separate outcomes.

| Evidence | What it resolves |
| --- | --- |
| Acquisition, relay receipt, per-branch publication/receipt times and drop counts | Did each condition create the intended outage while sensing continued upstream? |
| Odometry speed/yaw and command velocities before/after collision monitoring | Did the simulated robot actually stop or resume, and did upstream commands differ from final commands? |
| Timestamped map-to-odom transforms, receipt times and bounded lookup checks at relevant simulation times | Did transform availability/validity change alongside the abort? Account for AMCL's future-dated transforms; do not equate message receipt or pose publication rate with transform validity. |
| AMCL updates, transform errors, costmap warnings, collision-monitor state/warnings where supported | Which events occurred, and in what order? Verify installed state interfaces before relying on them. |
| Navigation feedback, recovery count and logs, terminal action result/error | Did the original goal survive, attempt recovery, abort or time out? Missing feedback is missing, not zero. |

Use Lab 02's stop definition: odometry translation <=0.01 m/s and absolute yaw rate <=0.03 rad/s for >=0.3 s, with sample gaps <=0.15 s. Resume requires translation >=0.05 m/s for >=0.3 s after the outage, following a measured stop. If no stop occurred, resume latency is not applicable. If no resumption occurred, report the observation endpoint rather than assigning zero. Odometry is simulated feedback, not independent physical ground truth.

Successful completion times may be shown as secondary context. Failed-task termination times must not be treated as completed journeys; normal timing spread is already large.

## Interpretation fixed before execution

- If A works, B reproduces the abort, and C keeps valid transforms and the original goal survives or succeeds after a stop, that supports the explanation that preserving AMCL input changes task survival in this setup. It does not prove the collision monitor alone caused the stop, or isolate the two costmaps.
- If C avoids the transform error but still fails or never resumes, report the changed failure mode and leave task recovery unresolved. Preserving localization may be insufficient.
- If C still produces transform errors despite verified fresh localization input, the simple AMCL-input explanation is insufficient; inspect the full transform chain and timing.
- If A is unreliable, B does not reproduce, or branch evidence fails, report that limitation before interpreting C. Do not pool historical Lab 02 trials with the new nine-trial sample as interchangeable controls.
- Mixed results remain mixed. No consumer-specific cause or fix is guaranteed in advance. This contrast estimates the effect of keeping one consumer supplied under the remaining outage; a reverse AMCL-only outage or full consumer separation would require a later protocol.

## Engineering value and eventual package

The practical lesson would be how to distinguish a stale-sensor motion stop from loss of localization transforms and an aborted mission. Engineers could adapt the procedure to check whether restored sensor messages are enough for their original task to continue, and to decide what evidence to capture before changing timeout settings. Its usefulness does not depend on obtaining a dramatic failure or a successful recovery.

After execution, prepare one all-trial outcome table and an aligned timeline of scan delivery by branch, transform validity, command/motion, and goal state. Retain protocol, exact versions/parameters, source hashes, all logs/CSVs and excluded attempts in a new archive. A short clip and reproduction package could support the series' public-awareness goal; publication remains a separate action. Keep the prior no-LinkedIn preference. No outbound messages or posts are authorized.

This remains a small standalone simulation associated with the founder's Vektor work. It does not integrate or evaluate Vektor, establish adoption or demand, show physical safety, or establish general Nav2 reliability. No experiment has been run for this plan.

## Technical basis and inspection

On 2026-10-18, read the KB startup pages, five latest log entries, all requested experiment/code/findings/report pages, and relevant attention/visibility guidance. Inspected live `code/robot-failure-lab/experiment02/run.py` and `trial.py` plus the archived parameters; no source code was edited or executed.

Primary references retrieved 2026-10-18:

- [Nav2 Jazzy AMCL configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/): configurable scan input, map/odometry transform broadcasting and transform tolerance.
- [Nav2 Jazzy AMCL source](https://api.nav2.org/nav2-jazzy/html/amcl__node_8cpp_source.html): the scan callback publishes/rebroadcasts the map-to-odom transform with expiration based on scan stamp plus tolerance. This motivates the hypothesis; it is not proof of the causal contribution in our runs. Verify the installed package revision during implementation.
- [Nav2 Jazzy collision-monitor configuration](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/): stale-source timeout behavior and configurable input/output command topics. Actual tested values come from the Lab 02 parameter snapshot, not documentation defaults.

Related: [robot-failure-lab-02-scan-blackout](<C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/wiki/experiments/robot-failure-lab-02-scan-blackout.md>), [robot-failure-lab](<C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/wiki/code/robot-failure-lab.md>), [robot-failure-lab-01b-repeatability](<C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/wiki/experiments/robot-failure-lab-01b-repeatability.md>), [robotics-attention-playbook-2026-09](<C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/wiki/analyses/robotics-attention-playbook-2026-09.md>), [nahid-public-visibility-restart-2026-10](<C:/Users/maxle/OneDrive/Documents/robotics-space/startup-kb/wiki/analyses/nahid-public-visibility-restart-2026-10.md>).
