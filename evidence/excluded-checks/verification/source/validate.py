"""Evidence checks independent of the ROS runtime and report renderer."""
import csv,json,hashlib
from collections import Counter
from core import forward_policy,routing_valid,window_metrics,TOPICS

def rows(path):
    with path.open() as f:
        return [{k:(int(v) if k=='scan_id' else (float(v) if v and k not in ('branch','state','polygon') else v)) for k,v in row.items()}
                for row in csv.DictReader(f)]

def validate_trial(root,record):
    trial=root/record['trial'];start=record['window_start_sim_s'];end=record['observation_end_sim_s']
    assert record['injection_valid'] and start is not None and end>=start+6
    assert routing_valid(record['graph']) and routing_valid(record['graph_end'])
    scans=rows(trial/'scans.csv');receipts=rows(trial/'branch-receipts.csv')
    odom=rows(trial/'odom.csv');commands=rows(trial/'commands.csv');feedback=rows(trial/'feedback.csv')
    assert len(scans)==record['scans_received'] and len({s['scan_id'] for s in scans})==len(scans)
    for s in scans:
        loc,obs=forward_policy(record['condition'],s['t'],start)
        assert bool(s['localization_forwarded'])==loc and bool(s['obstacles_forwarded'])==obs,'Gate mismatch'
        assert -.02<=s['t']-s['acquisition']<.5,'Unexpected scan acquisition age'
    inside=[s for s in scans if start<=s['t']<start+2]
    assert len(inside)>=5 and inside[0]['t']-start<.5 and start+2-inside[-1]['t']<.5
    assert all(b['t']-a['t']<.5 for a,b in zip(inside,inside[1:])),'Upstream sensing stalled'
    delivery={}
    scan_by_id={s['scan_id']:s for s in scans}
    for branch in TOPICS:
        field=branch+'_forwarded'
        branch_rx=[r for r in receipts if r['branch']==branch and r['scan_id'] in scan_by_id]
        counts=Counter(r['scan_id'] for r in branch_rx)
        assert all(n==1 for n in counts.values()),'Duplicate branch receipt'
        for r in branch_rx:
            s=scan_by_id[r['scan_id']]
            assert s[field] and r['acquisition']==s['acquisition'],'Delivered dropped or changed scan'
        # Exclude only the final 0.25s when shutting down could leave an output in transit.
        expected=[s for s in scans if s[field] and s['t']<end-.25]
        assert expected and all(counts[s['scan_id']]==1 for s in expected),'Missing branch receipt'
        after=[s for s in scans if s['t']>=start+2 and s[field]]
        before=[s for s in scans if s['t']<start and s[field]]
        assert before and after and after[0]['t']-(start+2)<.5,'No prompt restoration'
        dropped=sum(1-s[field] for s in scans)
        key='localization_scans_dropped' if branch=='localization' else 'obstacle_scans_dropped'
        assert dropped==record[key]
        delivery[branch]=dict(dropped=dropped,confirmed_receipts=len(branch_rx),
            restoration_after_window_s=after[0]['t']-(start+2))
    metrics=window_metrics(odom,commands,start,2,end)
    assert metrics==record['metrics']
    assert metrics['window_odom_samples']>=10 and metrics['window_odom_max_gap_s']<=.15
    assert metrics['pre_window_speed_mps']>=.04
    assert feedback and int(max(r['recoveries'] for r in feedback))==record['recoveries']
    assert rows(trial/'input-commands.csv') and rows(trial/'transforms.csv') and rows(trial/'localization.csv')
    probes=rows(trial/'transform-probes.csv')
    selected=[p for p in probes if start<=p['t']<=start+6]
    assert len(selected)>=60 and max(b['t']-a['t'] for a,b in zip(selected,selected[1:]))<=.2
    assert rows(trial/'goal-state.csv')[-1]['state']==record['outcome']
    assert record['goal_accepted'] and record['terminal_action_status'] in (4,5,6)
    # Full map-base lookups may miss freshly published odometry between callbacks;
    # record their evidence without treating every diagnostic miss as a task fault.
    summaries={}
    for label,lo,hi in [('before',start-.8,start),('window',start,start+2),('tail',start+2,start+6)]:
        group=[p for p in probes if lo<=p['t']<hi]
        summaries[label]=dict(samples=len(group),map_odom_unavailable=sum(not p['map_odom_ok'] for p in group),
            map_base_unavailable=sum(not p['map_base_ok'] for p in group))
    return dict(passed=True,branch_delivery=delivery,transform_probes=summaries,
        collision_state_events=len(rows(trial/'collision-state.csv')),
        note='Forwarding and independent observer receipts validated; not proof every Nav2 consumer processed every scan.')
