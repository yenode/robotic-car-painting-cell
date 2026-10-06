import argparse
import copy
import json
import math
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import CollisionObject, RobotState, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK, GetPositionFK, GetStateValidity, GetCartesianPath, GetMotionPlan
from control_msgs.action import FollowJointTrajectory
from .geometry import read_obj, make_strokes

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def pose(position, quaternion=(0., 0., 0., 1.)):
    return Pose(position=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])),
                orientation=Quaternion(x=float(quaternion[0]), y=float(quaternion[1]), z=float(quaternion[2]), w=float(quaternion[3])))


def state(names, positions):
    return RobotState(joint_state=JointState(name=list(names), position=[float(p) for p in positions]))


def seconds(duration):
    return duration.sec+duration.nanosec*1e-9


class Demo(Node):
    def __init__(self):
        super().__init__('painting_demo')
        self.declare_parameter('use_sim_time', True) if not self.has_parameter('use_sim_time') else None
        self.share = Path(get_package_share_directory('painting_cell'))
        self.config = yaml.safe_load((self.share/'config/cell.yaml').read_text())
        self.vertices, self.faces = read_obj(self.share/'models/hatchback/meshes/car_scaled.obj')
        self.strokes = make_strokes(self.vertices, self.faces, self.config)
        self.joints = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self._service_clients = {}
        for service_type, name in [(ApplyPlanningScene, 'apply_planning_scene'), (GetPositionIK, 'compute_ik'),
                (GetPositionFK, 'compute_fk'), (GetStateValidity, 'check_state_validity'),
                (GetCartesianPath, 'compute_cartesian_path'), (GetMotionPlan, 'plan_kinematic_path')]:
            client = self.create_client(service_type, '/'+name)
            if not client.wait_for_service(timeout_sec=45):
                raise RuntimeError(f'Missing service {name}; start cell.launch.py first')
            self._service_clients[name] = client
        self.action = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = self.create_publisher(MarkerArray, '/painting/markers', qos)
        self.spray = self.create_publisher(Bool, '/painting/spray_on', qos)
        self.spray.publish(Bool(data=False))
        self.report = {'status': 'running', 'robot': 'UR20', 'car_dimensions_m': [4.8, 1.9, 1.6],
            'car_xyz_m': self.config['car_xyz'],
            'base_xyz_m': self.config['robot_base'], 'standoff_m': self.config['standoff'],
            'layout_floor_m': self.config['layout']['overall_floor'],
            'layout_bay_m': self.config['layout']['central_bay'],
            'layout_collision_objects': [],
            'collision_sample_max_joint_step_rad': 0.04, 'strokes': [], 'executed_segments': []}

    def on_joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(j in values for j in JOINTS):
            self.joints = state(JOINTS, [values[j] for j in JOINTS])

    def wait(self, future, timeout=60):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError('ROS operation timed out')
        if future.exception():
            raise future.exception()
        return future.result()

    def call(self, name, request):
        return self.wait(self._service_clients[name].call_async(request))

    def current(self):
        deadline = time.monotonic()+20
        # Spin once even when cached data exists, so post-execution readings are fresh.
        rclpy.spin_once(self, timeout_sec=0.1)
        while self.joints is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joints is None:
            raise RuntimeError('No simulated joint feedback')
        return copy.deepcopy(self.joints)

    def add_scene(self):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        mesh = Mesh(vertices=[Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in self.vertices],
                    triangles=[MeshTriangle(vertex_indices=[int(i) for i in f]) for f in self.faces])
        car = CollisionObject(id='car', operation=CollisionObject.ADD)
        car.header.frame_id = 'world'
        car.meshes, car.mesh_poses = [mesh], [pose(self.config['car_xyz'])]
        req.scene.world.collision_objects.append(car)
        base = self.config['robot_base']
        floor = self.config['layout']['overall_floor']
        scene_boxes = [('floor', floor, [0., 0., -floor[2]/2]),
            ('pedestal', [0.45, 0.45, base[2]], [base[0], base[1], base[2]/2])]
        scene_boxes.extend((item['id'], item['size'], item['xyz'])
                           for item in self.config['layout']['boxes'] if item['collision'])
        for name, size, xyz in scene_boxes:
            obj = CollisionObject(id=name, operation=CollisionObject.ADD)
            obj.header.frame_id = 'world'
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
            obj.primitive_poses = [pose(xyz)]
            req.scene.world.collision_objects.append(obj)
        self.report['layout_collision_objects'] = [obj.id for obj in req.scene.world.collision_objects]
        if not self.call('apply_planning_scene', req).success:
            raise RuntimeError('Could not apply collision scene')
        self.show_paths()

    def show_paths(self):
        markers = []
        car = Marker(type=Marker.MESH_RESOURCE, action=Marker.ADD, id=0)
        car.header.frame_id = 'world'
        car.ns = 'cell'
        car.pose = pose(self.config['car_xyz'])
        car.scale.x = car.scale.y = car.scale.z = 1.
        car.color.r = car.color.g = car.color.b = 0.7
        car.color.a = 1.
        car.mesh_resource = 'file://'+str(self.share/'models/hatchback/meshes/car_scaled.obj')
        car.mesh_use_embedded_materials = True
        markers.append(car)
        for i, stroke in enumerate(self.strokes, 1):
            marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, id=i)
            marker.header.frame_id = 'world'
            marker.ns = 'painting_paths'
            marker.pose.orientation.w = 1.
            marker.scale.x = 0.012
            marker.color.a = 1.
            marker.color.r, marker.color.g, marker.color.b = {'hood': (1., 0.4, 0.1), 'roof': (0.1, 0.7, 1.), 'side': (0.3, 1., 0.3)}[stroke['panel']]
            marker.points = [pose(p).position for p in stroke['positions']]
            markers.append(marker)
        self._base_markers = markers
        self.markers.publish(MarkerArray(markers=markers))

    def spray_status(self, enabled, label):
        self.spray.publish(Bool(data=enabled))
        marker = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=100)
        marker.header.frame_id = 'world'
        marker.ns = 'status'
        marker.pose = pose([0., 0., 2.4])
        marker.scale.z = 0.16
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.2, 1., 0.3, 1.) if enabled else (1., 0.8, 0.2, 1.)
        marker.text = f'{label} | SPRAY {"ON" if enabled else "OFF"}'
        cone = self.spray_cone(enabled)
        # Keep geometry in every latched message so late RViz subscribers see the complete cell.
        self.markers.publish(MarkerArray(markers=self._base_markers+[marker, cone]))

    @staticmethod
    def spray_cone(enabled):
        cone = Marker(type=Marker.TRIANGLE_LIST,
                      action=Marker.ADD if enabled else Marker.DELETE,
                      id=101)
        cone.header.frame_id = 'spray_nozzle'
        cone.ns = 'spray_visualization'
        cone.pose.orientation.w = 1.
        cone.color.r, cone.color.g, cone.color.b, cone.color.a = 1.0, 0.45, 0.05, 0.30
        length, radius, sectors = 0.25, 0.10, 24
        apex = Point(x=0., y=0., z=0.)
        for index in range(sectors):
            a0 = 2*math.pi*index/sectors
            a1 = 2*math.pi*(index+1)/sectors
            cone.points.extend([
                apex,
                Point(x=radius*math.cos(a0), y=radius*math.sin(a0), z=length),
                Point(x=radius*math.cos(a1), y=radius*math.sin(a1), z=length),
            ])
        return cone

    def valid(self, robot_state):
        req = GetStateValidity.Request(robot_state=robot_state, group_name='ur_manipulator')
        return self.call('check_state_validity', req)

    def ik(self, target, seed, collision=True):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.ik_link_name = 'spray_nozzle'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        req.ik_request.pose_stamped.pose = target
        req.ik_request.robot_state = seed
        req.ik_request.avoid_collisions = collision
        req.ik_request.timeout.sec = 2
        response = self.call('compute_ik', req)
        if response.error_code.val != 1:
            raise RuntimeError(f'IK failed ({response.error_code.val}) at {target.position}')
        seed_values = dict(zip(seed.joint_state.name, seed.joint_state.position))
        values = list(response.solution.joint_state.position)
        for i, name in enumerate(response.solution.joint_state.name):
            if name in seed_values and name != 'elbow_joint':
                candidates = [values[i]+k*2*math.pi for k in range(-2, 3)]
                candidates = [q for q in candidates if -2*math.pi <= q <= 2*math.pi]
                values[i] = min(candidates, key=lambda q: abs(q-seed_values[name]))
        response.solution.joint_state.position = values
        return response.solution

    def transition(self, start, goal):
        req = GetMotionPlan.Request()
        plan = req.motion_plan_request
        plan.group_name, plan.pipeline_id, plan.planner_id = 'ur_manipulator', 'ompl', 'RRTConnect'
        plan.start_state = start
        plan.allowed_planning_time, plan.num_planning_attempts = 10., 3
        plan.max_velocity_scaling_factor = plan.max_acceleration_scaling_factor = 0.15
        plan.goal_constraints = [Constraints(joint_constraints=[JointConstraint(joint_name=j, position=float(q),
            tolerance_above=0.0001, tolerance_below=0.0001, weight=1.) for j, q in zip(goal.joint_state.name, goal.joint_state.position) if j in JOINTS])]
        result = self.call('plan_kinematic_path', req).motion_plan_response
        if result.error_code.val != 1:
            raise RuntimeError(f'Transition planning failed: {result.error_code.val}')
        return result.trajectory.joint_trajectory

    def cartesian(self, start, stroke):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state, req.group_name, req.link_name = start, 'ur_manipulator', 'spray_nozzle'
        req.waypoints = [pose(p, stroke['quaternion']) for p in stroke['positions']]
        req.max_step, req.revolute_jump_threshold = 0.01, 0.3
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.15
        result = self.call('compute_cartesian_path', req)
        if result.error_code.val != 1 or result.fraction < 0.999:
            raise RuntimeError(f'{stroke["name"]}: incomplete Cartesian path {result.fraction:.3f}, code {result.error_code.val}')
        trajectory = result.solution.joint_trajectory
        length = float(np.linalg.norm(np.diff(stroke['positions'], axis=0), axis=1).sum())
        duration = seconds(trajectory.points[-1].time_from_start)
        if duration <= 0:
            raise RuntimeError('Cartesian trajectory has no timing')
        factor = max(1., (length/self.config['stroke_speed'])/duration)
        for point in trajectory.points:
            ns = round(seconds(point.time_from_start)*factor*1e9)
            point.time_from_start = Duration(sec=ns//10**9, nanosec=ns%10**9)
            point.velocities = [v/factor for v in point.velocities]
            point.accelerations = [a/factor**2 for a in point.accelerations]
        return trajectory, result.fraction

    def validate_trajectory(self, trajectory):
        count = 0
        previous = None
        for point in trajectory.points:
            current = np.asarray(point.positions)
            samples = [current] if previous is None else np.linspace(previous, current,
                max(2, math.ceil(float(np.abs(current-previous).max())/0.04)+1))[1:]
            for sample in samples:
                result = self.valid(state(trajectory.joint_names, sample))
                count += 1
                if not result.valid:
                    contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
                    raise RuntimeError(f'Trajectory collision/constraint failure: {contacts}')
            previous = current
        return count

    def negative_check(self, seed):
        stroke = self.strokes[0]
        target = pose(stroke['surface'][0]-0.03*stroke['normal'], stroke['quaternion'])
        invalid = self.ik(target, seed, collision=False)
        result = self.valid(invalid)
        contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
        detected = not result.valid and any('car' in pair for pair in contacts)
        self.report['negative_collision_test'] = {'passed': detected, 'contacts': contacts, 'executed': False}
        if not detected:
            raise RuntimeError(f'Negative collision test failed: {contacts}')

    def execute(self, trajectory, name, spray):
        if not self.action.wait_for_server(timeout_sec=15):
            raise RuntimeError('Trajectory controller not available')
        current = dict(zip(self.current().joint_state.name, self.current().joint_state.position))
        mismatch = max(abs(current[j]-q) for j, q in zip(trajectory.joint_names, trajectory.points[0].positions))
        if mismatch > 0.03:
            raise RuntimeError(f'{name}: current state differs from planned start by {mismatch:.3f} rad')
        self.spray_status(spray, name)
        handle = self.wait(self.action.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory)))
        if not handle.accepted:
            raise RuntimeError('Controller rejected trajectory')
        result_future = handle.get_result_async()
        try:
            result = self.wait(result_future, timeout=max(60., 3*seconds(trajectory.points[-1].time_from_start)+15))
        except Exception:
            self.wait(handle.cancel_goal_async(), timeout=10)
            raise
        finally:
            self.spray_status(False, name)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError(f'{name}: controller execution failed: {result.result.error_string}')
        actual = self.current()
        values = dict(zip(actual.joint_state.name, actual.joint_state.position))
        error = max(abs(values[j]-q) for j, q in zip(trajectory.joint_names, trajectory.points[-1].positions))
        if error > 0.03:
            raise RuntimeError(f'{name}: joint endpoint error {error:.4f} rad')
        self.report['executed_segments'].append({'name': name, 'spray_on': spray, 'duration_s': seconds(trajectory.points[-1].time_from_start), 'max_endpoint_joint_error_rad': error})

    def run(self, execute=False):
        self.add_scene()
        current = self.current()
        check = self.valid(current)
        if not check.valid:
            raise RuntimeError(f'Initial state invalid: {[(c.contact_body_1,c.contact_body_2) for c in check.contacts]}')
        self.negative_check(current)
        sequence = []
        for stroke in self.strokes:
            self.get_logger().info(f'Planning {stroke["name"]}')
            # An endpoint may have multiple IK branches; require a continuous full stroke
            # before accepting a branch, then plan the spray-OFF approach to that branch.
            seeds = [current] + [state(JOINTS, angles) for angles in [
                [-2.5, -2., -1., -1.5, 1.5, 0.],
                [0.6, -1.3, 1.4, -1.7, -1.5, 0.],
                [-2.5, -3., 1.2, -2.7, 1.7, -0.8],
                [-1.5, -1.5, 1.5, -1.5, -1.5, 0.],
                [1.5, -1.5, -1.5, -1.5, 1.5, 0.]]]
            failures = []
            for seed in seeds:
                try:
                    goal = self.ik(pose(stroke['positions'][0], stroke['quaternion']), seed)
                    self.cartesian(goal, stroke)
                    transfer = self.transition(current, goal)
                    transfer_count = self.validate_trajectory(transfer)
                    start = state(transfer.joint_names, transfer.points[-1].positions)
                    painting, fraction = self.cartesian(start, stroke)
                    break
                except RuntimeError as error:
                    failures.append(str(error))
            else:
                raise RuntimeError(f'{stroke["name"]}: no continuous reachable branch: {failures}')
            paint_count = self.validate_trajectory(painting)
            distances = np.linalg.norm(stroke['positions']-stroke['surface'], axis=1)
            if not np.allclose(distances, self.config['standoff'], atol=1e-8):
                raise RuntimeError(f'{stroke["name"]}: TCP standoff construction changed unexpectedly')
            self.report['strokes'].append({'name': stroke['name'], 'cartesian_fraction': fraction,
                'rejected_ik_branches': len(failures),
                'transition_collision_samples': transfer_count, 'painting_collision_samples': paint_count,
                'maximum_surface_normal_deviation_deg': stroke['normal_error_deg'],
                'target_standoff_min_m': float(distances.min()),
                'target_standoff_max_m': float(distances.max()),
                'nozzle_start_xyz_m': stroke['positions'][0].tolist(), 'nozzle_end_xyz_m': stroke['positions'][-1].tolist()})
            sequence.extend([(transfer, stroke['name']+'_approach', False), (painting, stroke['name'], True)])
            current = state(painting.joint_names, painting.points[-1].positions)
        self.report['status'] = 'validated'
        if execute:
            for trajectory, name, spray in sequence:
                self.get_logger().info(f'Executing {name}')
                self.execute(trajectory, name, spray)
            self.report['status'] = 'executed'
        self.spray_status(False, self.report['status'].upper())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true', help='Execute validated trajectories in the running simulation')
    parser.add_argument('--output', default='results/validation.json')
    parser.add_argument('--keep-alive', action='store_true', help='Keep RViz markers available after completion')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = None
    try:
        node = Demo()
        node.run(args.execute)
    except Exception as error:
        if node:
            node.report['status'], node.report['error'] = 'failed', str(error)
        raise
    finally:
        if node:
            output = Path(args.output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(node.report, indent=2)+'\n')
            node.spray.publish(Bool(data=False))
            node.get_logger().info(f'Report saved to {output}')
    try:
        if args.keep_alive:
            rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
