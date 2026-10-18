#!/usr/bin/env python3
"""One fresh Nav2 trial; telemetry persists after terminal task state."""
import argparse,csv,json,math,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped,Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
from nav2_msgs.msg import CollisionMonitorState
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from core import CONDITIONS,TOPICS,forward_policy,routing_valid,window_metrics

class Observer(Node):
    def __init__(self,condition):
        super().__init__('failure_lab_03_observer')
        self.set_parameters([Parameter('use_sim_time',value=True)])
        self.condition=condition
        self.lock=threading.RLock()
        self.window=None
        self.trigger_pose=None
        self.scans=[];self.receipts=[];self.odom=[];self.commands=[];self.input_commands=[]
        self.transforms=[];self.probes=[];self.localization=[];self.collision=[]
        self.thread_error=None
        self.scan_publishers={branch:self.create_publisher(LaserScan,topic,qos_profile_sensor_data) for branch,topic in TOPICS.items()}
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        for branch,topic in TOPICS.items():
            self.create_subscription(LaserScan,topic,lambda msg,b=branch:self.receipt(msg,b),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.motion,qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.command(m,self.commands),qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel_smoothed',lambda m:self.command(m,self.input_commands),qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf',self.tf,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.pose,qos_profile_sensor_data)
        self.create_subscription(CollisionMonitorState,'/collision_monitor_state',self.state,10)
        self.buffer=Buffer(cache_time=Duration(seconds=20.0))
        self.listener=TransformListener(self.buffer,self,spin_thread=False)
        self.create_timer(0.05,self.probe)

    def now(self): return self.get_clock().now().nanoseconds/1e9
    @staticmethod
    def stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    @staticmethod
    def scan_id(msg): return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec

    def scan(self,msg):
        with self.lock:
            now=self.now()
            loc,obs=forward_policy(self.condition,now,self.window)
            if loc:self.scan_publishers['localization'].publish(msg)
            if obs:self.scan_publishers['obstacles'].publish(msg)
            self.scans.append(dict(t=now,acquisition=self.stamp(msg),scan_id=self.scan_id(msg),
                localization_forwarded=int(loc),obstacles_forwarded=int(obs)))

    def receipt(self,msg,branch):
        with self.lock:
            self.receipts.append(dict(t=self.now(),branch=branch,scan_id=self.scan_id(msg),acquisition=self.stamp(msg)))

    def motion(self,msg):
        with self.lock:
            p,v=msg.pose.pose.position,msg.twist.twist
            self.odom.append(dict(t=self.now(),speed=math.hypot(v.linear.x,v.linear.y),w=v.angular.z,x=p.x,y=p.y))

    def command(self,msg,destination):
        with self.lock:
            destination.append(dict(t=self.now(),speed=math.hypot(msg.linear.x,msg.linear.y),w=msg.angular.z))

    def tf(self,msg):
        with self.lock:
            for tr in msg.transforms:
                if tr.header.frame_id.lstrip('/')=='map' and tr.child_frame_id.lstrip('/')=='odom':
                    self.transforms.append(dict(t=self.now(),stamp=self.stamp(tr),x=tr.transform.translation.x,y=tr.transform.translation.y))

    def probe(self):
        # Zero-duration lookup: diagnostics never wait for transforms to arrive.
        with self.lock:
            now=self.now();target=Time(seconds=now,clock_type=self.get_clock().clock_type)
            mo=bool(self.buffer.can_transform('odom','map',target,timeout=Duration()))
            mb=bool(self.buffer.can_transform('map','base_link',target,timeout=Duration()))
            latest=self.transforms[-1]['stamp'] if self.transforms else None
            self.probes.append(dict(t=now,map_odom_ok=int(mo),map_base_ok=int(mb),
                latest_map_odom_stamp=latest,valid_until_minus_now=None if latest is None else latest-now))

    def pose(self,msg):
        with self.lock:
            self.localization.append(dict(t=self.now(),stamp=self.stamp(msg),x=msg.pose.pose.position.x,y=msg.pose.pose.position.y))

    def state(self,msg):
        with self.lock:
            self.collision.append(dict(t=self.now(),action_type=int(msg.action_type),polygon=msg.polygon_name))

    def graph(self):
        return {topic:[dict(node=s.node_name,namespace=s.node_namespace) for s in self.get_subscriptions_info_by_topic(topic)]
                for topic in ('/scan',*TOPICS.values())}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--condition',choices=CONDITIONS,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=90)
    args=parser.parse_args()
    rclpy.init()
    observer=Observer(args.condition)
    executor=SingleThreadedExecutor();executor.add_node(observer)
    def spin():
        try: executor.spin()
        except Exception as error: observer.thread_error=repr(error)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    nav=BasicNavigator();nav.set_parameters([Parameter('use_sim_time',value=True)])
    def pose(x,y):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=nav.get_clock().now().to_msg()
        p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.0
        return p
    nav.setInitialPose(pose(-2.,-.5));nav.waitUntilNav2Active()
    settle=observer.now()
    while observer.now()-settle<1.5:rclpy.spin_once(nav,timeout_sec=.05)
    graph=observer.graph()
    if not routing_valid(graph):raise RuntimeError(f'Unexpected scan routing: {graph}')
    if not all(any(r['branch']==b for r in observer.receipts) for b in TOPICS):
        raise RuntimeError('Both scan branches must be delivering before goal submission')
    (args.output/'READY.json').write_text(json.dumps(dict(sim_time=observer.now(),graph=graph),indent=2))
    start,wall=observer.now(),time.monotonic()
    (args.output/'GOAL_SUBMITTED.json').write_text(json.dumps(dict(t=start,goal=[1.,.5,0.])))
    accepted=bool(nav.goToPose(pose(1.,.5)))
    feedback_rows=[];goals=[dict(t=observer.now(),state='accepted' if accepted else 'rejected')]
    moving_since=None;timed_out=False
    while accepted and not nav.isTaskComplete():
        now=observer.now()
        if observer.thread_error:raise RuntimeError(observer.thread_error)
        with observer.lock:
            last=observer.odom[-1] if observer.odom else None
            if last and now-last['t']<=.15 and last['speed']>=.05:
                moving_since=now if moving_since is None else moving_since
            else:moving_since=None
            if observer.window is None and now-start>=3 and moving_since is not None and now-moving_since>=.3:
                observer.window=now;observer.trigger_pose=dict(x=last['x'],y=last['y'])
                print(f'Window starts at goal +{now-start:.3f}s',flush=True)
        feedback=nav.getFeedback()
        if feedback:
            p=feedback.current_pose.pose.position
            feedback_rows.append(dict(t=now,x=p.x,y=p.y,recoveries=int(feedback.number_of_recoveries)))
        goals.append(dict(t=now,state='active'))
        if now-start>=args.timeout or time.monotonic()-wall>=270:
            timed_out=True;nav.cancelTask();break
    end=observer.now()
    result=nav.getResult() if accepted else TaskResult.FAILED
    response=nav.result_future.result() if accepted and nav.result_future.done() else None
    action_result=response.result if response else None
    outcome='timeout' if timed_out else ('succeeded' if result==TaskResult.SUCCEEDED else 'failed')
    goals.append(dict(t=end,state=outcome))
    window=observer.window
    if window is not None:
        tail_wall=time.monotonic()
        while observer.now()<window+6:
            if time.monotonic()-tail_wall>30:raise RuntimeError('Simulation stalled in observation tail')
            if observer.thread_error:raise RuntimeError(observer.thread_error)
            rclpy.spin_once(nav,timeout_sec=.05)
    observation_end=observer.now()
    graph_end=observer.graph()
    executor.shutdown();thread.join(timeout=3)
    if observer.thread_error:raise RuntimeError(observer.thread_error)
    def save(name,rows,fields):
        rows=[r for r in rows if start<=r['t']<=observation_end]
        with (args.output/name).open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        return rows
    scans=save('scans.csv',observer.scans,['t','acquisition','scan_id','localization_forwarded','obstacles_forwarded'])
    save('branch-receipts.csv',observer.receipts,['t','branch','scan_id','acquisition'])
    odom=save('odom.csv',observer.odom,['t','speed','w','x','y'])
    commands=save('commands.csv',observer.commands,['t','speed','w'])
    save('input-commands.csv',observer.input_commands,['t','speed','w'])
    save('transforms.csv',observer.transforms,['t','stamp','x','y'])
    save('transform-probes.csv',observer.probes,['t','map_odom_ok','map_base_ok','latest_map_odom_stamp','valid_until_minus_now'])
    save('localization.csv',observer.localization,['t','stamp','x','y'])
    save('collision-state.csv',observer.collision,['t','action_type','polygon'])
    save('feedback.csv',feedback_rows,['t','x','y','recoveries'])
    save('goal-state.csv',goals,['t','state'])
    metrics=window_metrics(odom,commands,window,2,observation_end) if window is not None else {}
    record=dict(condition=args.condition,duration_s=2.,start_sim_s=start,end_sim_s=end,
        window_start_sim_s=window,window_start_task_s=None if window is None else window-start,
        trigger_pose=observer.trigger_pose,outcome=outcome,goal_accepted=accepted,
        terminal_action_status=getattr(response,'status',None),
        elapsed_sim_s=end-start,elapsed_wall_s=time.monotonic()-wall,observation_end_sim_s=observation_end,
        action_error_code=getattr(action_result,'error_code',None),action_error_msg=getattr(action_result,'error_msg',None),
        scans_received=len(scans),localization_scans_dropped=sum(1-r['localization_forwarded'] for r in scans),
        obstacle_scans_dropped=sum(1-r['obstacles_forwarded'] for r in scans),
        recoveries=max((r['recoveries'] for r in feedback_rows),default=None),
        graph=graph,graph_end=graph_end,metrics=metrics,
        injection_valid=window is not None and observation_end>=window+6 and routing_valid(graph_end))
    (args.output/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)
    nav.destroy_node();observer.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
