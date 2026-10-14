"""RViz 목표 수신 -> 목표 이동 -> 사진 저장 -> 출발 위치 복귀."""
import copy
import math
import threading
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from modules.base_bt_nodes import (
    BTNodeList, Fallback, Node, ReactiveFallback, ReactiveSequence,
    Sequence, Status, SyncCondition,
)
from modules.base_bt_nodes_ros import (
    ActionWithROSAction, ActionWithROSService, ConditionWithROSTopics,
)
from modules.utils import config

CUSTOM_ACTION_NODES = [
    "SaveInitialPose", "MoveToGoal", "CaptureImage", "Return", "FinishMission",
]
CUSTOM_CONDITION_NODES = [
    "WaitForGoal", "HasInitialPose", "IsGoalReached", "IsImageCaptured", "IsReturned",
]
for registry, names in (
    (BTNodeList.ACTION_NODES, CUSTOM_ACTION_NODES),
    (BTNodeList.CONDITION_NODES, CUSTOM_CONDITION_NODES),
):
    for node_name in names:
        if node_name not in registry:
            registry.append(node_name)


def normalized_pose(pose):
    result = copy.deepcopy(pose)
    p, q = result.pose.position, result.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Position and orientation must contain finite values")
    length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if length < 1e-12:
        raise ValueError("Orientation quaternion has zero length")
    q.x, q.y, q.z, q.w = q.x / length, q.y / length, q.z / length, q.w / length
    return result


class MissionContext:
    """여러 BT 노드가 한 작업의 목표, 출발 위치, 완료 상태를 공유한다."""
    def __init__(self, agent):
        self.node = agent.ros_bridge.node
        self.options = config.get("photo_mission", {})
        self.global_frame = self.options.get("global_frame", "map")
        self.base_frame = self.options.get("base_frame", "base_link")
        self.goal_topic = self.options.get("goal_topic", "/bt/goal_pose")
        self.navigation_action = self.options.get("navigation_action", "/navigate_to_pose")
        self.capture_service = self.options.get("capture_service", "/capture_image")
        for key in (
            "pose_wait_timeout", "server_wait_timeout",
            "navigation_timeout", "capture_timeout",
        ):
            value = float(self.options.get(key, {
                "pose_wait_timeout": 30.0, "server_wait_timeout": 10.0,
                "navigation_timeout": 180.0, "capture_timeout": 10.0,
            }[key]))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be positive and finite")
            self.options[key] = value

        result = self.node.set_parameters([
            Parameter("use_sim_time", value=bool(self.options.get("use_sim_time", True)))
        ])
        if not result[0].successful:
            raise RuntimeError(f"Could not configure simulation time: {result[0].reason}")
        self.tf_buffer = Buffer(node=self.node)
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

        self.lock = threading.RLock()
        self.active = False
        self.mission_id = 0
        self.target_pose = None
        self.initial_pose = None
        self.goal_reached = False
        self.image_captured = False
        self.returned = False
        self.photo_path = ""
        self._notices = {}
        self.node.get_logger().info(f"Waiting for an RViz goal on {self.goal_topic}")

    def begin(self, goal):
        with self.lock:
            self.active = True
            self.mission_id += 1
            self.target_pose = goal
            self.initial_pose = None
            self.goal_reached = self.image_captured = self.returned = False
            self.photo_path = ""
        p = goal.pose.position
        self.node.get_logger().info(
            f"Mission {self.mission_id}: goal received, x={p.x:.3f}, y={p.y:.3f}"
        )

    def fail(self, reason):
        with self.lock:
            self.active = False
        self.node.get_logger().error(f"Mission {self.mission_id} failed: {reason}")
        self.node.get_logger().info(f"Waiting for a new RViz goal on {self.goal_topic}")

    def notice(self, key, message, interval=3.0):
        now = time.monotonic()
        if now - self._notices.get(key, -math.inf) >= interval:
            self._notices[key] = now
            self.node.get_logger().warn(message)

    def pose_for_goal(self, key):
        source = getattr(self, key)
        if source is None:
            return None
        result = copy.deepcopy(source)
        result.header.frame_id = self.global_frame
        result.header.stamp = self.node.get_clock().now().to_msg()
        return result


def mission_context(agent):
    if not hasattr(agent, "_photo_mission_context"):
        agent._photo_mission_context = MissionContext(agent)
    return agent._photo_mission_context


class WaitForGoal(ConditionWithROSTopics):
    def __init__(self, name, agent):
        # 독자 콜백으로 작업 중 목표 변경을 차단하므로 기본 구독 목록은 비워 둔다.
        super().__init__(name, agent, [])
        self.context = mission_context(agent)
        self.goal_sub = self.ros.node.create_subscription(
            PoseStamped, self.context.goal_topic, self._on_goal, 10
        )

    def _on_goal(self, msg):
        try:
            if msg.header.frame_id != self.context.global_frame:
                raise ValueError(
                    f"Goal frame must be {self.context.global_frame}, "
                    f"received {msg.header.frame_id!r}"
                )
            goal = normalized_pose(msg)
        except ValueError as error:
            self.ros.node.get_logger().warn(f"Ignored invalid goal: {error}")
            return
        with self.context.lock:
            if self.context.active or "goal" in self._cache:
                self.ros.node.get_logger().warn(
                    "Mission already active; send the next goal after return completes"
                )
                return
            self._cache["goal"] = goal

    def _predicate(self, agent, blackboard):
        with self.context.lock:
            goal = self._cache.pop("goal", None)
            if goal is None:
                return False
            self.context.begin(goal)
        blackboard["mission_id"] = self.context.mission_id
        return True


class SaveInitialPose(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Action"
        self.context = mission_context(agent)
        self._started = None

    async def run(self, agent, blackboard):
        if self.context.initial_pose is not None:
            self.status = Status.SUCCESS
            return self.status
        if self._started is None:
            self._started = time.monotonic()
        try:
            transform = self.context.tf_buffer.lookup_transform(
                self.context.global_frame, self.context.base_frame, Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = self.context.global_frame
            pose.header.stamp = self.context.node.get_clock().now().to_msg()
            t = transform.transform.translation
            pose.pose.position.x, pose.pose.position.y = t.x, t.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = copy.deepcopy(transform.transform.rotation)
            self.context.initial_pose = normalized_pose(pose)
        except TransformException:
            if time.monotonic() - self._started > self.context.options["pose_wait_timeout"]:
                self.context.fail(
                    "No map-to-robot TF. Set the initial pose with RViz 2D Pose Estimate"
                )
                self.status = Status.FAILURE
            else:
                self.context.notice(
                    "initial_pose",
                    f"Waiting for TF {self.context.global_frame} -> "
                    f"{self.context.base_frame}; check RViz 2D Pose Estimate",
                )
                self.status = Status.RUNNING
            return self.status
        except Exception as error:
            self.context.fail(f"Could not save the initial pose: {error}")
            self.status = Status.FAILURE
            return self.status

        p = self.context.initial_pose.pose.position
        blackboard["initial_pose"] = copy.deepcopy(self.context.initial_pose)
        self.context.node.get_logger().info(
            f"Initial pose saved: x={p.x:.3f}, y={p.y:.3f} (orientation also saved)"
        )
        self.status = Status.SUCCESS
        return self.status

    def halt(self):
        self._started = None


class _FlagCondition(SyncCondition):
    flag = ""

    def __init__(self, name, agent):
        self.context = mission_context(agent)
        super().__init__(name, self._check)

    def _check(self, agent, blackboard):
        return Status.SUCCESS if bool(getattr(self.context, self.flag)) else Status.FAILURE


class HasInitialPose(_FlagCondition):
    flag = "initial_pose"


class IsGoalReached(_FlagCondition):
    flag = "goal_reached"


class IsImageCaptured(_FlagCondition):
    flag = "image_captured"


class IsReturned(_FlagCondition):
    flag = "returned"


class _NavigationAction(ActionWithROSAction):
    """강의의 Action Client를 사용하며 매 tick마다 future의 완료 여부를 확인한다."""
    pose_key = ""
    done_flag = ""

    def __init__(self, name, agent):
        self.context = mission_context(agent)
        super().__init__(
            name, agent, (NavigateToPose, self.context.navigation_action)
        )
        self._goal_future = None
        self._wait_started = None
        self._started = None

    def _build_goal(self, agent, blackboard):
        pose = self.context.pose_for_goal(self.pose_key)
        if pose is None:
            return None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            setattr(self.context, self.done_flag, True)
            self.ros.node.get_logger().info(f"{self.name}: succeeded")
            return Status.SUCCESS
        self.context.fail(f"{self.name}: navigation ended with status {status_code}")
        return Status.FAILURE

    def _fail(self, reason):
        self.halt()
        self.context.fail(f"{self.name}: {reason}")
        self.status = Status.FAILURE
        return self.status

    async def run(self, agent, blackboard):
        now = time.monotonic()
        try:
            if self._phase == "idle":
                if self._wait_started is None:
                    self._wait_started = now
                if not self.client.wait_for_server(timeout_sec=0.0):
                    if now - self._wait_started > self.context.options["server_wait_timeout"]:
                        return self._fail("Nav2 action server is unavailable")
                    self.context.notice(self.name, "Waiting for Nav2 action server")
                    self.status = Status.RUNNING
                    return self.status

                goal = self._build_goal(agent, blackboard)
                if goal is None:
                    return self._fail("Navigation pose is missing")
                self._started = now
                self._phase = "sending"
                self._goal_future = self.client.send_goal_async(goal)
                p = goal.pose.pose.position
                self.ros.node.get_logger().info(
                    f"{self.name}: goal sent, x={p.x:.3f}, y={p.y:.3f}"
                )
                self.status = Status.RUNNING
                return self.status

            if self._phase == "sending" and self._goal_future.done():
                self._goal_handle = self._goal_future.result()
                if self._goal_handle is None or not self._goal_handle.accepted:
                    return self._fail("Nav2 rejected the goal; check Nav2 and localization")
                self._result_future = self._goal_handle.get_result_async()
                self._phase = "running"

            if self._phase == "running" and self._result_future.done():
                result = self._result_future.result()
                self.status = self._interpret_result(
                    result.result, agent, blackboard, result.status
                )
                self._phase = "done"
                return self.status

            if self._started is not None:
                if now - self._started > self.context.options["navigation_timeout"]:
                    return self._fail("Navigation timed out")
            self.status = Status.RUNNING
            return self.status

        except Exception as error:
            return self._fail(str(error))

    @staticmethod
    def _cancel_late_goal(future):
        try:
            handle = future.result()
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
        except Exception:
            pass

    def halt(self):
        if self._phase in ("sending", "running"):
            if self._goal_handle is not None:
                try:
                    self._goal_handle.cancel_goal_async()
                except Exception:
                    pass
            elif self._goal_future is not None:
                self._goal_future.add_done_callback(self._cancel_late_goal)
        self._phase = "idle"
        self._goal_future = self._goal_handle = self._result_future = None
        self._wait_started = self._started = None


class MoveToGoal(_NavigationAction):
    pose_key = "target_pose"
    done_flag = "goal_reached"


class Return(_NavigationAction):
    pose_key = "initial_pose"
    done_flag = "returned"


class CaptureImage(ActionWithROSService):
    def __init__(self, name, agent):
        self.context = mission_context(agent)
        super().__init__(name, agent, (Trigger, self.context.capture_service))
        self._started = None

    def _build_request(self, agent, blackboard):
        self.ros.node.get_logger().info("CaptureImage: requesting photo storage")
        return Trigger.Request()

    def _interpret_response(self, response, agent, blackboard):
        if response is None or not response.success:
            detail = response.message if response is not None else "Empty service response"
            self.context.fail(f"CaptureImage: {detail}")
            return Status.FAILURE
        path = Path(response.message)
        if not path.is_file() or path.stat().st_size == 0:
            self.context.fail(f"CaptureImage: saved file cannot be found: {path}")
            return Status.FAILURE

        self.context.photo_path = str(path)
        self.context.image_captured = True
        blackboard["photo_path"] = str(path)
        self.ros.node.get_logger().info(f"CaptureImage: saved {path}")
        return Status.SUCCESS

    async def run(self, agent, blackboard):
        if self._started is None:
            self._started = time.monotonic()
        try:
            status = await super().run(agent, blackboard)
            if status == Status.RUNNING:
                if time.monotonic() - self._started > self.context.options["capture_timeout"]:
                    self.context.fail("CaptureImage: service request timed out")
                    self.halt()
                    status = Status.FAILURE
            self.status = status
            return status
        except Exception as error:
            self.context.fail(f"CaptureImage: {error}")
            self.halt()
            self.status = Status.FAILURE
            return self.status

    def halt(self):
        super().halt()
        self._future = None
        self._started = None


class FinishMission(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Action"
        self.context = mission_context(agent)

    async def run(self, agent, blackboard):
        if not all((
            self.context.goal_reached,
            self.context.image_captured,
            self.context.returned,
        )):
            self.context.fail("Cannot finish an incomplete mission")
            self.status = Status.FAILURE
            return self.status

        blackboard["last_mission"] = {
            "mission_id": self.context.mission_id,
            "photo_path": self.context.photo_path,
            "initial_pose": copy.deepcopy(self.context.initial_pose),
            "target_pose": copy.deepcopy(self.context.target_pose),
        }
        with self.context.lock:
            self.context.active = False
        self.context.node.get_logger().info(
            f"Mission {self.context.mission_id} complete: returned to the initial pose"
        )
        self.context.node.get_logger().info(
            f"Waiting for a new RViz goal on {self.context.goal_topic}"
        )
        self.status = Status.SUCCESS
        return self.status
