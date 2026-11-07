import math 
import threading
import time 
import rclpy
from builtin_interfaces.msg import Duration 
from control_msgs.action import FollowJointTrajectory 
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory,MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject,Constraints,JointConstraint,
                               OrientationConstraint, PlanningScene, PositionConstraint)

from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient 
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node 
from sensor_msgs.msg import JointState 
from shape_msgs.msg import  SolidPrimitive 
from std_msgs.msg import String 
from trajectory_msgs.msg  import JointTrajectory, JointTrajectoryPoint 
from vision_msgs.msg import Detection3DArray 

BASE,TIP ='panda_link0','panda_hand_tcp'
JOINTS = [f'panda_joint{i}' for i in range(1,8)]
HOME = [0.0,-0.785,0.0,-2.356,0.0,1.571,0.785]
BINS ={'red':(0.25,0.45),'green':(0.40,0.45),'blue':(0.55,0.45)}
PRE,DROP,TRAVEL,CLOSED=0.12,0.14,0.25,0.012   # CLOSED < cube half-width (0.02): fingers squeeze, and an empty grasp closes further
COLORS = {'R':'red','G':'green','B':'blue'}


def wait(f,timeout=60.0):
    t=time.time()
    while not f.done() and time.time() - t< timeout:
        time.sleep(0.01)
    return f.done()

class Sorter(Node):
    def __init__(self):
        super().__init__('sorter')
        g=ReentrantCallbackGroup()
        self.move = ActionClient(self,MoveGroup,'move_action',callback_group=g)
        self.exe = ActionClient(self,ExecuteTrajectory,'execute_trajectory',callback_group=g)
        self.hand = ActionClient(self,FollowJointTrajectory,'panda_hand_controller/follow_joint_trajectory',callback_group=g)
        self.cart = self.create_client(GetCartesianPath,'compute_cartesian_path',callback_group=g)
        self.scene = self.create_publisher(PlanningScene,'planning_scene',10)
        self.dets,self.skip,self.finger,self.finger2,self.active =[] ,{},None ,None, set(COLORS.values())
        self.create_subscription(Detection3DArray,'detections',self.on_dets,10,callback_group=g)
        self.create_subscription(JointState,'joint_states',self.on_js,10,callback_group=g)
        self.create_subscription(String,'sorter/command',self.on_cmd,10,callback_group=g)
        threading.Thread(target=self.run,daemon=True).start()



    def pose(self,xyz,q):
        p = Pose()
        p.position.x,p.position.y,p.position.z = map(float,xyz)
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = map(float,q)
        return p

    def on_dets(self,m):
        now=time.time()
        self.dets= [(d.id, d.results[0].hypothesis.class_id,d.results[0].pose.pose,now) for d in m.detections]


    def on_js(self,m):
        if 'panda_finger_joint1' in m.name and 'panda_finger_joint2' in m.name:
            self.finger = m.position[m.name.index('panda_finger_joint1')]
            self.finger2 = m.position[m.name.index('panda_finger_joint2')]

    def on_cmd(self,m):
        c= m.data.strip().upper()
        self.active =set() if c=='STOP' else set(COLORS.values()) if c=='ALL' else {COLORS[x] for x in c if x in COLORS}
        self.get_logger().info(f'active colours:{sorted(self.active)}')

    def send(self ,client,goal,timeout=60.0):
        f= client.send_goal_async(goal)
        if not wait(f,10) or not f.result().accepted:
            return None
        r= f.result().get_result_async()
        return r.result().result if  wait(r,timeout) else None

    def plan(self,cons):
        goal =MoveGroup.Goal()
        r=goal.request 
        r.group_name,r.pipeline_id, r.planner_id = "panda_arm","ompl","RRTConnectkConfigDefault"
        r.num_planning_attempts,r.allowed_planning_time=5,5.0 
        r.max_velocity_scaling_factor=r.max_acceleration_scaling_factor=0.5
        r.start_state.is_diff =True 
        r.goal_constraints =[cons]
        goal.planning_options.planning_scene_diff.is_diff=True
        goal.planning_options.planning_scene_diff.robot_state.is_diff=True
        res= self.send(self.move,goal)
        return bool(res) and res.error_code.val ==1 

    def to_joints(self,q):
        return self.plan(Constraints(joint_constraints=[
            JointConstraint(joint_name=n,position=v,tolerance_above=0.005,tolerance_below=0.005,weight=1.0)
            for n, v in zip(JOINTS,q)
        ]))
        

    def to_pose(self,xyz,q):
        p,h=self.pose (xyz,q),PoseStamped().header 
        h.frame_id =BASE
        pc = PositionConstraint(header=h,link_name=TIP,weight=1.0,constraint_region=BoundingVolume(
            primitives=[SolidPrimitive(type =SolidPrimitive.SPHERE,dimensions=[0.005])],primitive_poses=[p]

        ))
        oc =OrientationConstraint(header=h,link_name=TIP,orientation=p.orientation,weight=1.0,
                                  absolute_x_axis_tolerance=0.05,absolute_y_axis_tolerance=0.05,absolute_z_axis_tolerance=0.05)
        return self.plan(Constraints(position_constraints=[pc],orientation_constraints=[oc]))



    def line(self,xyz,q):
        req=GetCartesianPath.Request()
        req.header.frame_id,req.group_name,req.link_name=BASE,"panda_arm",TIP
        req.start_state.is_diff,req.waypoints =True,[self.pose(xyz,q)]
        req.max_step,req.avoid_collisions =0.005,True
        for k in ('max_velocity_scaling_factor','max_acceleration_scaling_factor'):
            if hasattr(req,k):
                setattr(req,k,0.3)     # slow, gentle descent / lift near the cube
        f = self.cart.call_async(req)
        if not wait (f,10) or f.result().fraction<0.95:
            self.get_logger().warn(f'cartesian path to {tuple(round(v,3) for v in xyz)} only '
                                   f'{f.result().fraction*100 if f.done() else 0:.0f}% feasible')
            return False 
        res = self.send(self.exe,ExecuteTrajectory.Goal(trajectory=f.result().solution))
        return bool (res) and  res.error_code.val==1 

    def grip(self,w,check=True):
        pt = JointTrajectoryPoint(positions=[w,w],time_from_start=Duration(sec=1))
        res = self.send(self.hand,FollowJointTrajectory.Goal(
            trajectory=JointTrajectory(joint_names=["panda_finger_joint1","panda_finger_joint2"],points=[pt])
        ),10)
        if not check:      # closing on a cube: the cube blocks the fingers, so the controller reports a
            return res is not None   # goal-tolerance error. That is the SUCCESS case; held() decides.
        return bool(res) and res.error_code==0

    def held(self):
        time.sleep(0.5)
        if self.finger is None or self.finger2 is None:
            return True
        w = self.finger + self.finger2      # opening between the pads: cube = 0.040, empty grasp = 2*CLOSED = 0.024
        self.get_logger().info(f'grip width {w*1000:.1f} mm')
        return 0.034 < w < 0.048

    def run(self):
        while not (self.move.wait_for_server(2) and self.exe.wait_for_server(2) and self.hand.wait_for_server(2)
                   and self.cart.wait_for_service(2)):
            self.get_logger().info('waiting for MoveIt / controllers...')
        co=CollisionObject(id='table',operation=CollisionObject.ADD)
        co.header.frame_id=BASE
        co.primitives =[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[1.4,1.6,0.05])]
        co.primitive_poses =[self.pose((0.35,0,-0.06),(0,0,0,1))]
        sc=PlanningScene(is_diff=True)
        sc.world.collision_objects=[co]
        time.sleep(1.0)
        self.scene.publish(sc)
        self.grip(0.04)
        self.to_joints(HOME)
        while rclpy.ok():
            now=time.time()
            ok =[d for d in self.dets if d[1] in self.active and now - d[3] < 1.5 and self.skip.get(d[0],0)<now]
            if not ok:
                time.sleep(0.5)
                continue 
            tid,color,p,_=min(ok,key=lambda d: math.hypot(d[2].position.x,d[2].position.y))
            if self.pick_place(color,p):
                self.get_logger().info(f'{tid}-> {color} bin')
            else:
                self.skip[tid]=time.time()+20
                self.get_logger().warn(f'{tid} failed,skipping for 20s')
                self.grip(0.04)
                self.to_joints(HOME)


    def pick_place(self,color,p):
        phi =2 * math.atan2(p.orientation.z,p.orientation.w)
        yaw = phi + math.pi/ 2 
        yaw -= math.pi if yaw > math.pi / 2 else 0 
        q= (math.cos(yaw/2), math.sin(yaw/2),0.0,0.0)
        down =(1.0,0.0,0.0,0.0)
        x,y,z=p.position.x,p.position.y,p.position.z + 0.005
        bx,by = BINS[color]
        steps =[('open',lambda:self.grip(0.04)),
                ('pre-grasp',lambda:self.to_pose((x,y,z+PRE),q)),
                ('descend',lambda:self.line((x,y,z),q)),
                ('close',lambda:self.grip(CLOSED,check=False)),
                ('grip check',self.held),
                ('lift',lambda:self.line((x,y,z+PRE),q)),
                ('to bin',lambda:self.to_pose((bx,by,TRAVEL),down)),
                ('lower',lambda:self.line((bx,by,DROP),down)),
                ('release',lambda:self.grip(0.04)),
                ('raise',lambda:self.line((bx,by,TRAVEL),down)),
                ('home',lambda:self.to_joints(HOME))]
        for name,fn in steps:
            if not fn():
                self.get_logger().warn(f'{color}: step "{name}" failed')
                return False
        return True


def main():
    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=6)
    node = Sorter()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

        
        
