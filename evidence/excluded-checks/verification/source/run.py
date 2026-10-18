#!/usr/bin/env python3
"""Bounded three-condition experiment with immutable source/evidence snapshots."""
import argparse,hashlib,json,os,shutil,signal,subprocess,sys,time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from core import CONDITIONS,routed_config
from validate import validate_trial

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def stop(process):
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=12)
    except subprocess.TimeoutExpired:pass
    try:os.killpg(process.pid,signal.SIGKILL)
    except ProcessLookupError:pass
    process.wait(timeout=5)

def schedule(verification):
    if verification:return [(0,c) for c in CONDITIONS]
    return [(b+1,c) for b in range(3) for c in CONDITIONS[b:]+CONDITIONS[:b]]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    parser.add_argument('--verification',action='store_true')
    parser.add_argument('--verification-root',type=Path)
    parser.add_argument('--resume-infrastructure',action='store_true')
    parser.add_argument('--domain',type=int,default=191)
    args=parser.parse_args()
    if not 1<=args.domain<=220:parser.error('Invalid ROS domain')
    root=args.output.resolve();snapshot=root/'source'
    if args.resume_infrastructure:
        protocol=json.loads((root/'PROTOCOL.json').read_text())
        assert protocol['verification']==args.verification
        for name,digest in protocol['source_sha256'].items():assert sha(snapshot/name)==digest
        records=json.loads((root/'results.json').read_text())
        continuation=root/f'continuation-{time.time_ns()}.json'
        continuation.write_text(json.dumps(dict(command=sys.argv,domain=args.domain,
            preserved_trials=[r['trial'] for r in records],runner_sha256=sha(Path(__file__)),
            reason='Continue only before-goal infrastructure failures. Never rerun task outcomes.'),indent=2))
    else:
        root.mkdir(parents=True,exist_ok=False);snapshot.mkdir()
        parent=Path(__file__).resolve().parent
        for path in [*parent.glob('*.py'),parent/'README.md']:shutil.copy2(path,snapshot/path.name)
        bringup=Path(get_package_share_directory('nav2_bringup'))
        upstream=bringup/'launch/tb3_simulation_launch.py';launch=upstream.read_text()
        needle="cmd=['gz', 'sim', '-r', '-s', world_sdf]"
        assert launch.count(needle)==1,'Installed Gazebo launch changed'
        shutil.copy2(upstream,snapshot/'upstream_tb3_simulation_launch.txt')
        (snapshot/'seeded_simulation_launch.py').write_text(launch.replace(needle,
            "cmd=['gz', 'sim', '--seed', os.environ['RFL_GZ_SEED'], '-r', '-s', world_sdf]"))
        config=yaml.safe_load((bringup/'params/nav2_params.yaml').read_text())
        routed=routed_config(config)
        (root/'params.yaml').write_text(yaml.safe_dump(routed,sort_keys=False))
        (root/'upstream-params.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        for name in ('tb3_sandbox.pgm','tb3_sandbox.yaml'):shutil.copy2(bringup/'maps'/name,root/name)
        versions=subprocess.run(['dpkg-query','-W','ros-jazzy-nav2-*','ros-jazzy-ros-gz-*','gz-*'],capture_output=True,text=True)
        (root/'packages.txt').write_text(versions.stdout+versions.stderr)
        previous=parent.parents[2]/'artifacts/robot-failure-lab-02'
        # parent is .../code/robot-failure-lab/experiment03; parents[2] is KB root.
        old_params=yaml.safe_load((previous/'params.yaml').read_text())
        def normalize(v):
            if isinstance(v,dict):return {k:('/scan' if k in ('topic','scan_topic') and isinstance(x,str) and x.startswith('/failure_lab') else normalize(x)) for k,x in v.items()}
            if isinstance(v,list):return [normalize(x) for x in v]
            return v
        comparison=dict(parameters_equal_except_scan_routes=normalize(routed)==normalize(old_params),
            packages_identical=(root/'packages.txt').read_text()==(previous/'packages.txt').read_text())
        (root/'lab02-environment-comparison.json').write_text(json.dumps(comparison,indent=2))
        assert all(comparison.values()),'Environment changed; review before execution'
        if not args.verification:
            if args.verification_root is None:raise RuntimeError('Main trials require verified excluded checks')
            vr=args.verification_root.resolve();vp=json.loads((vr/'PROTOCOL.json').read_text())
            checks=json.loads((vr/'results.json').read_text())
            assert len(checks)==3 and {r['condition'] for r in checks}==set(CONDITIONS)
            for record in checks:validate_trial(vr,record)
            # Frozen measurement/runner code must match the verified code.
            for path in parent.glob('*.py'):assert sha(path)==vp['source_sha256'][path.name],path.name
            assert sha(root/'params.yaml')==sha(vr/'params.yaml')
        protocol=dict(experiment='Robot Failure Lab 03',verification=args.verification,
            verification_root=str(args.verification_root) if args.verification_root else None,
            schedule=[dict(block=b,condition=c,gazebo_seed=100+b) for b,c in schedule(args.verification)],
            trigger='>=3 sim seconds after goal submission; translation >=0.05m/s sustained >=0.3s; odom age <=0.15s',
            window_s=2.,post_window_min_observation_s=4.,equal_pre_goal_wait_sim_s=1.5,
            task_timeout_sim_s=90.,task_timeout_wall_s=270.,startup_timeout_wall_s=60.,
            change='Two identical scan branches. Full outage drops both; localization-fed drops only obstacle branch; normal drops neither. No buffering/replay/timestamp rewriting.',
            seeds='Gazebo 101/102/103 per main block; verification seed100. AMCL and scheduling uncontrolled. Not deterministic trajectories.',
            collision_monitor_source_timeout_s=routed['collision_monitor']['ros__parameters']['source_timeout'],
            stop='odom translation <=0.01m/s and |yaw| <=0.03rad/s for >=0.3s; gaps <=0.15s',
            resume='translation >=0.05m/s for >=0.3s after window following measured stop',
            scope='Standalone simulation; no replacement goal, Vektor integration, physical safety or general reliability claim.',
            command=sys.argv,platform=os.uname().release,python=sys.version,
            source_sha256={p.name:sha(p) for p in snapshot.iterdir()},
            config_sha256=sha(root/'params.yaml'),packages_sha256=sha(root/'packages.txt'))
        (root/'PROTOCOL.json').write_text(json.dumps(protocol,indent=2)+'\n')
        records=[];(root/'results.json').write_text('[]\n')
    for entry in protocol['schedule']:
        block,condition,seed=entry['block'],entry['condition'],entry['gazebo_seed']
        prior=[r for r in records if r['block']==block and r['condition']==condition]
        if prior:
            assert len(prior)==1 and prior[0]['evidence_validated'];continue
        base=f'block-{block}-{condition}';name=base;trial=root/name;attempt=1
        while trial.exists():
            if (not args.resume_infrastructure or not (trial/'INFRASTRUCTURE_ERROR.txt').exists()
                    or any((trial/f).exists() for f in ('READY.json','GOAL_SUBMITTED.json','result.json'))):
                raise RuntimeError(f'Refusing to replace/rerun evidence: {trial}')
            attempt+=1
            if attempt>3:raise RuntimeError('Three startup attempts exhausted')
            name=f'{base}-attempt-{attempt}';trial=root/name
        trial.mkdir()
        env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
            GZ_PARTITION=f'rfl03-{os.getpid()}-{name}',LIBGL_ALWAYS_SOFTWARE='1',RFL_GZ_SEED=str(seed))
        env.pop('ROS_LOCALHOST_ONLY',None)
        command=['ros2','launch',str(snapshot/'seeded_simulation_launch.py'),'headless:=True',
            'use_rviz:=False','use_composition:=False',f'params_file:={root / "params.yaml"}']
        (trial/'launch-command.json').write_text(json.dumps(dict(command=command,gazebo_seed=seed,domain=args.domain,
            host_load=os.getloadavg(),params_sha256=sha(root/'params.yaml')),indent=2))
        print(f'Starting {name}',flush=True);sim=worker=None
        try:
            with (trial/'simulation.log').open('w') as sl,(trial/'trial.log').open('w') as tl:
                sim=subprocess.Popen(command,env=env,stdout=sl,stderr=subprocess.STDOUT,start_new_session=True)
                time.sleep(8)
                worker=subprocess.Popen([sys.executable,str(snapshot/'trial.py'),'--condition',condition,'--output',str(trial)],
                    env=env,stdout=tl,stderr=subprocess.STDOUT,start_new_session=True)
                began=time.monotonic()
                while worker.poll() is None:
                    elapsed=time.monotonic()-began
                    if elapsed>60 and not (trial/'READY.json').exists():raise TimeoutError('Readiness exceeded 60 wall seconds')
                    if elapsed>370:raise TimeoutError('Whole-worker deadline exceeded')
                    time.sleep(.2)
                if worker.returncode or not (trial/'result.json').exists():raise RuntimeError('Worker error; inspect retained logs')
        except BaseException as error:
            (trial/'INFRASTRUCTURE_ERROR.txt').write_text(repr(error)+'\n');raise
        finally:
            if worker is not None:stop(worker)
            if sim is not None:stop(sim)
        record=json.loads((trial/'result.json').read_text())
        record.update(trial=name,block=block,gazebo_seed=seed,evidence_validated=False)
        records.append(record);(root/'results.json').write_text(json.dumps(records,indent=2)+'\n')
        try:
            validation=validate_trial(root,record)
            (trial/'VALIDATION.json').write_text(json.dumps(validation,indent=2)+'\n')
            record['evidence_validated']=True
            (root/'results.json').write_text(json.dumps(records,indent=2)+'\n')
        except BaseException as error:
            (trial/'PROTOCOL_ERROR.txt').write_text(repr(error)+'\n');raise
        print(f'{name}: {record["outcome"]} in {record["elapsed_sim_s"]:.3f} sim s; error={record["action_error_code"]}; metrics={record["metrics"]}',flush=True)
    print(f'COMPLETE: {len(records)} validated trials',flush=True)

if __name__=='__main__':main()
