from pathlib import Path
import json,re,sys
root=Path(sys.argv[1]);audit=[]
for path in sorted(root.glob('block-*')):
    if not path.is_dir():continue
    sim=path/'simulation.log'
    lines=sim.read_text(errors='replace').splitlines() if sim.exists() else []
    crashes=[(i,line) for i,line in enumerate(lines) if 'process has died' in line]
    shutdown=[i for i,line in enumerate(lines) if 'user interrupted with ctrl-c' in line or 'signal_handler(SIGINT' in line]
    row=dict(trial=path.name,result_exists=(path/'result.json').exists(),
        ready_exists=(path/'READY.json').exists(),goal_submitted=(path/'GOAL_SUBMITTED.json').exists(),
        infrastructure_error=(path/'INFRASTRUCTURE_ERROR.txt').exists(),protocol_error=(path/'PROTOCOL_ERROR.txt').exists(),
        process_deaths=[dict(line_index=i,text=line,after_shutdown_signal=bool(shutdown and i>min(shutdown))) for i,line in crashes])
    audit.append(row)
(root/'ATTEMPT-AUDIT.json').write_text(json.dumps(audit,indent=2)+'\n')
for row in audit:
    if row['result_exists']:
        assert all(d['after_shutdown_signal'] for d in row['process_deaths']),row['trial']
print(json.dumps(dict(attempts=len(audit),recorded_task_results=sum(r['result_exists'] for r in audit),
    pre_goal_infrastructure_failures=[r['trial'] for r in audit if r['infrastructure_error'] and not r['goal_submitted']],
    segmentation_faults_after_shutdown=[r['trial'] for r in audit if any('exit code -11' in d['text'] and d['after_shutdown_signal'] for d in r['process_deaths'])]),indent=2))
