#!/usr/bin/env python3
"""Revalidate all measured records, retain an all-run table and two figures."""
import argparse,hashlib,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from core import CONDITIONS
from validate import rows,validate_trial

LABELS={'normal':'Normal','full-outage':'Full outage','localization-fed':'Localization fed'}
COLORS={'normal':'#16829a','full-outage':'#c65c35','localization-fed':'#6352aa'}

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);args=p.parse_args();root=args.root
    records=json.loads((root/'results.json').read_text());protocol=json.loads((root/'PROTOCOL.json').read_text())
    assert len(records)==len(protocol['schedule'])
    assert [(r['block'],r['condition']) for r in records]==[(s['block'],s['condition']) for s in protocol['schedule']]
    for name,digest in protocol['source_sha256'].items():
        assert hashlib.sha256((root/'source'/name).read_bytes()).hexdigest()==digest
    validations={r['trial']:validate_trial(root,r) for r in records}
    (root/'EVIDENCE-VALIDATION.json').write_text(json.dumps(validations,indent=2)+'\n')
    fmt=lambda v:'—' if v is None else f'{v:.3f}'
    lines=['# Robot Failure Lab 03: keeping localization fed during a scan outage','',
        'Measured standalone Gazebo/Nav2 simulation. No Vektor integration or physical validation.','',
        '## Every planned trial','',
        '| Block | Condition | Original goal outcome | Termination time (sim s) | Error | Stop after onset (s) | Resume after window (s) | Drops: localization / obstacles | Recoveries |',
        '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in records:
        m=r['metrics']
        lines.append(f'| {r["block"]} | {LABELS[r["condition"]]} | {r["outcome"]} | {r["elapsed_sim_s"]:.3f} | {r["action_error_code"]} | {fmt(m["stop_latency_s"])} | {fmt(m["resume_after_window_s"])} | {r["localization_scans_dropped"]} / {r["obstacle_scans_dropped"]} | {r["recoveries"]} |')
    lines+=['','A failed task terminating early did not complete the route faster. A dash is no qualifying event, not zero latency. Resume is only defined after a measured stop.','',
        '## Condition summaries','']
    for c in CONDITIONS:
        g=[r for r in records if r['condition']==c]
        lines.append(f'- {LABELS[c]}: {sum(r["outcome"]=="succeeded" for r in g)}/{len(g)} original goals succeeded; '
            f'{sum(r["metrics"]["stop_latency_s"] is not None for r in g)}/{len(g)} stopped in the window; '
            f'{sum(r["metrics"]["resume_after_window_s"] is not None for r in g)}/{len(g)} showed post-window resumption after a stop.')
    lines+=['','## Transform diagnostics','',
        'Zero-duration probes ask whether map/odom and map/base transforms can be looked up at the current simulated time. These are diagnostic samples, not independent navigation failures. Map/base may miss between odometry callbacks. AMCL map/odom headers are future-dated; their timestamp minus current time measures remaining time coverage, not pose accuracy.','',
        '| Trial | Window map/odom unavailable / probes | Tail map/odom unavailable / probes | Original goal at end of outage |',
        '| --- | --- | --- | --- |']
    for r in records:
        v=validations[r['trial']]['transform_probes'];w,t=v['window'],v['tail']
        at_end='active' if r['end_sim_s']>r['window_start_sim_s']+2 else r['outcome']
        lines.append(f'| {r["trial"]} | {w["map_odom_unavailable"]} / {w["samples"]} | {t["map_odom_unavailable"]} / {t["samples"]} | {at_end} |')
    lines+=['','## Method, evidence and limits','',
        '- Three main blocks, Gazebo seeds 101/102/103. Orders normal/full/localization-fed, full/localization-fed/normal, localization-fed/normal/full. Three excluded verification trials precede the main series; all attempts are retained separately.',
        '- Fresh simulation/Nav2 for each trial; equal 1.5-sim-second pre-goal wait. The same two scan branches and all navigation parameters are used in every condition. Full outage gates both branches; localization-fed gates only the branch for both costmaps and collision monitor.',
        '- Trigger after >=3 sim seconds and translation >=0.05 m/s sustained >=0.3 s with fresh odometry. Drop newly received scans for two simulated seconds; no buffering, replay, or acquisition timestamp rewriting. Collision monitor remains enabled with unchanged one-second source timeout.',
        '- Stop: odometry translation <=0.01 m/s and |yaw rate| <=0.03 rad/s for >=0.3 s, with <=0.15-s sample gaps. Resume: translation >=0.05 m/s for >=0.3 s after the window, following a measured stop. Report interval-start times after confirming duration.',
        '- Observe through at least four simulated seconds after the outage, even after an action abort; continue active tasks to terminal result or 90-sim/270-wall-second task limit. No replacement goal, reset or altered transform timeout. Startup limit 60 wall seconds; infrastructure errors remain separate from task outcomes.',
        '- All records are revalidated scan by scan for gate policy, upstream continuity, acquisition age, branch receipt, prompt restoration, routing, motion sampling and recomputed metrics. Independent observer receipt is not proof each Nav2 consumer processed every scan. Source hashes, frozen parameters and versions are retained.',
        '- AMCL randomness, host scheduling, exact paths and trigger positions remain uncontrolled. Matching Gazebo seeds does not make trajectories deterministic. Odometry is simulated feedback, not independent physical ground truth. Three runs per condition on one static route do not establish a population failure rate, physical safety or general Nav2 reliability.',
        '- The contrast changes AMCL input availability under the remaining outage. It does not separately isolate both costmaps and collision monitor, establish a production remedy for a failed scanner, resolve Lab01 scan-delay uncertainty, or evaluate Vektor. Publication/outbound messages have not occurred.','',
        '![All trial outcomes](comparison.png)','', '![First block aligned evidence](first-block-timeline.png)','']
    (root/'REPORT.md').write_text('\n'.join(lines))
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
    fig,axes=plt.subplots(1,2,figsize=(12,5.5),layout='constrained')
    for r in records:
        c=r['condition'];color=COLORS[c];x=r['block']+(CONDITIONS.index(c)-1)*.23;m=r['metrics']
        axes[0].scatter(x,r['elapsed_sim_s'],c=color,s=65,marker='o' if r['outcome']=='succeeded' else 'X')
        text=f'{r["elapsed_sim_s"]:.2f}' + ('' if r['outcome']=='succeeded' else '\nfailed')
        axes[0].annotate(text,(x,r['elapsed_sim_s']),xytext=(0,8),textcoords='offset points',ha='center',fontsize=8)
        if m['stop_latency_s'] is not None:axes[1].scatter(x,m['stop_latency_s'],c=color,s=60,marker='s')
        if m['resume_after_window_s'] is not None:axes[1].scatter(x,m['resume_after_window_s'],c=color,s=70,marker='^')
        elif m['stop_latency_s'] is not None:axes[1].annotate('no resume',(x,2.12),rotation=45,ha='center',fontsize=7,color=color)
        else:axes[1].annotate('no stop',(x,.08),rotation=45,ha='center',fontsize=7,color=color)
    for c in CONDITIONS:axes[0].plot([],[],'o',color=COLORS[c],label=LABELS[c])
    axes[0].plot([],[],'X',color='#333',label='Failed original goal')
    axes[0].legend(frameon=False,fontsize=8,loc='upper left')
    axes[0].set_ylabel('Goal submission to termination (sim s)')
    axes[0].set_ylim(0,max(r['elapsed_sim_s'] for r in records)*1.22)
    axes[0].set_title('Every task outcome; failed times are not completed journeys')
    axes[1].plot([],[],'s',color='#333',label='Stop after outage onset')
    axes[1].plot([],[],'^',color='#333',label='Resume after outage end')
    axes[1].legend(frameon=False,fontsize=8,loc='upper left')
    axes[1].set_ylim(0,2.8);axes[1].set_ylabel('Latency from the stated reference (sim s)')
    axes[1].set_title('Measured motion: sustained intervals >=0.3 s')
    blocks=sorted({r['block'] for r in records})
    for ax in axes:ax.set_xticks(blocks);ax.set_xlim(min(blocks)-.5,max(blocks)+.5);ax.set_xlabel('Block / matched Gazebo seed; other randomness uncontrolled');ax.grid(alpha=.2)
    fig.suptitle('Robot Failure Lab 03 · Does preserving localization change task survival?\nMeasured Gazebo/Nav2 simulation · standalone runner')
    fig.savefig(root/'comparison.png',dpi=170);plt.close(fig)
    first=min(r['block'] for r in records)
    chosen=sorted([r for r in records if r['block']==first],key=lambda r:CONDITIONS.index(r['condition']))
    fig,axes=plt.subplots(3,4,figsize=(17,9),layout='constrained',sharex=True)
    for i,r in enumerate(chosen):
        folder=root/r['trial'];start=r['window_start_sim_s'];color=COLORS[r['condition']]
        scans=rows(folder/'scans.csv');probes=rows(folder/'transform-probes.csv');odom=rows(folder/'odom.csv')
        for field,y,label in [('localization_forwarded',1,'AMCL'),('obstacles_forwarded',0,'Others')]:
            for value,marker,col in [(1,'|',color),(0,'x','#222')]:
                selected=[s for s in scans if bool(s[field])==bool(value)]
                axes[i,0].scatter([s['t']-start for s in selected],[y]*len(selected),marker=marker,color=col,s=16)
        axes[i,0].set_yticks([0,1],['Others','AMCL']);axes[i,0].set_ylim(-.5,1.5)
        axes[i,0].set_ylabel(LABELS[r['condition']],color=color,fontweight='bold')
        valid=[p for p in probes if p['valid_until_minus_now']!='']
        axes[i,1].plot([p['t']-start for p in valid],[p['valid_until_minus_now'] for p in valid],color=color)
        miss=[p for p in probes if not p['map_odom_ok']]
        axes[i,1].scatter([p['t']-start for p in miss],[0]*len(miss),color='#b83030',s=8)
        axes[i,1].axhline(0,color='#666',lw=.7);axes[i,1].set_ylabel('TF valid-until minus now (s)')
        axes[i,1].set_ylim(-1.5,1.5)
        axes[i,2].plot([o['t']-start for o in odom],[o['speed'] for o in odom],color=color,label='Odometry')
        for name,ls,label in [('input-commands.csv','--','Monitor input command'),('commands.csv',':','Final command')]:
            data=rows(folder/name);axes[i,2].plot([d['t']-start for d in data],[d['speed'] for d in data],ls=ls,lw=1,alpha=.8,label=label)
        axes[i,2].set_ylabel('Translation (m/s)');axes[i,2].set_ylim(-.015,.32)
        if i==0:axes[i,2].legend(frameon=False,fontsize=7)
        task_end=r['end_sim_s']-start
        axes[i,3].plot([-1,min(task_end,6)],[1,1],color=color,lw=3)
        if task_end<=6:
            axes[i,3].plot([task_end,task_end,6],[1,0,0],color=color,lw=2)
            axes[i,3].text(task_end+.12,.2,r['outcome']+'\nerror '+str(r['action_error_code']),fontsize=9)
        else:axes[i,3].text(.03,.25,f'Active through shown window\nFinal: {r["outcome"]}\nGoal-to-end: {r["elapsed_sim_s"]:.3f} sim s',transform=axes[i,3].transAxes,fontsize=9)
        axes[i,3].set_yticks([0,1],['Terminal','Active']);axes[i,3].set_ylim(-.3,1.3)
        for ax in axes[i]:ax.axvspan(0,2,color='#e8b68c',alpha=.25);ax.set_xlim(-1,6);ax.grid(alpha=.15)
    for j,title in enumerate(['Scans: | forwarded, x dropped','Map/odom coverage; red = lookup miss','Commands and simulated motion','Original goal; no replacement']):axes[0,j].set_title(title,fontsize=10)
    for ax in axes[-1]:ax.set_xlabel('Sim seconds from each trial\'s trigger')
    fig.suptitle('Predeclared first block · Scan branches, transforms, motion and original goal\nShading: two-second window; transform probes are diagnostics, not additional task failures',fontsize=13)
    fig.savefig(root/'first-block-timeline.png',dpi=160);plt.close(fig)
    print(f'Validated {len(records)} trials; saved REPORT.md and two figures')

if __name__=='__main__':main()
