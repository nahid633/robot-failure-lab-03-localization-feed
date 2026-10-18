"""Pure gate, routing and motion measurements for Experiment 03."""
import math

CONDITIONS=('normal','full-outage','localization-fed')
TOPICS={'localization':'/failure_lab03/localization_scan',
        'obstacles':'/failure_lab03/obstacle_scan'}

def forward_policy(condition, now, start, duration=2.0):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    inside=start is not None and start <= now < start+duration
    return (not (inside and condition=='full-outage'),
            not (inside and condition!='normal'))

def routed_config(config):
    def route(value):
        if isinstance(value,dict):
            return {k:(TOPICS['obstacles'] if k in ('topic','scan_topic') and v in ('scan','/scan')
                       else route(v)) for k,v in value.items()}
        if isinstance(value,list): return [route(v) for v in value]
        return value
    result=route(config)
    result['amcl']['ros__parameters']['scan_topic']=TOPICS['localization']
    return result

def routing_valid(graph, observer='failure_lab_03_observer'):
    names=lambda topic:{s['node'] for s in graph.get(topic,[])}
    # The observer subscribes to both outputs to independently record receipt.
    return (names('/scan')=={observer}
            and names(TOPICS['localization'])=={observer,'amcl'}
            and names(TOPICS['obstacles'])=={observer,'local_costmap','global_costmap','collision_monitor'})

def sustained(rows,predicate,after,before,duration=0.3,max_gap=0.15):
    start=previous=None
    for row in rows:
        t=row['t']
        if t<after or t>before: continue
        if previous is not None and t-previous>max_gap: start=None
        if predicate(row):
            start=t if start is None else start
            if t-start>=duration-1e-8: return start
        else: start=None
        previous=t
    return None

def window_metrics(odom,commands,start,duration,observation_end):
    end=start+duration
    stopped=lambda r:r['speed']<=0.01 and abs(r['w'])<=0.03
    moving=lambda r:r['speed']>=0.05
    stop=sustained(odom,stopped,start,end)
    command_stop=sustained(commands,stopped,start,end)
    resume=sustained(odom,moving,end,observation_end) if stop is not None else None
    before=[r for r in odom if start-0.3<=r['t']<start]
    during=[r for r in odom if start<=r['t']<=end]
    return dict(stop_latency_s=None if stop is None else stop-start,
        zero_command_latency_s=None if command_stop is None else command_stop-start,
        resume_after_window_s=None if resume is None else resume-end,
        pre_window_speed_mps=sum(r['speed'] for r in before)/len(before) if before else None,
        window_path_m=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(during,during[1:])),
        window_odom_samples=len(during),
        window_odom_max_gap_s=max((b['t']-a['t'] for a,b in zip(during,during[1:])),default=None))
