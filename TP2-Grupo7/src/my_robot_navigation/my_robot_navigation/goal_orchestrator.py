import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from rclpy.executors import MultiThreadedExecutor


import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath

class GoalOrchestrator(Node):

    def __init__(self):
        super().__init__('goal_orchestrator')

        # --- Parametros configurables ---
        self.declare_parameter('replan_period', 3.0)
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('controller_id', 'FollowPath')       # FollowPath=DWB, FollowPathRPP=RPP
        self.declare_parameter('goal_checker_id', 'general_goal_checker')

        self.replan_period = self.get_parameter('replan_period').value
        self.planner_id = self.get_parameter('planner_id').value
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value

        # --- TF: para consultar la pose actual del robot (map -> base_link) ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Action clients ---
        cb_group = ReentrantCallbackGroup()
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose',
            callback_group=cb_group)
        self.follow_path_client = ActionClient(
            self, FollowPath, 'follow_path',
            callback_group=cb_group)

        # --- Suscripcion a la meta publicada por "2D Goal Pose" de RViz ---
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10,
            callback_group=cb_group)

        # --- Estado interno ---
        self.current_goal = None            # PoseStamped de la meta activa
        self.follow_path_goal_handle = None
        self.compute_path_goal_handle = None
        self.replan_timer = None
        self.state = 'IDLE'                 # IDLE | PLANNING | NAVIGATING

        self.follow_path_generation = 0   # contador para descartar resultados de goals viejos (reemplazados por un replan)


        self.get_logger().info('Orquestador listo. Esperando metas en /goal_pose...')


    def get_robot_pose(self):
        """Consulta la pose actual del robot via TF (map -> base_link).
        Devuelve un PoseStamped, o None si la transformada no esta disponible."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except TransformException as ex:
            self.get_logger().warn(f'No se pudo obtener la pose del robot (TF): {ex}')
            return None

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = tf.header.stamp
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose


    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'Meta nueva recibida: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        if self.state != 'IDLE':
            self.get_logger().info('Cancelando navegacion en curso para atender la meta nueva...')
            self.cancel_active_navigation()

        self.current_goal = msg
        self.start_navigation()


    def cancel_active_navigation(self):
        if self.replan_timer is not None:
            self.replan_timer.cancel()
            self.replan_timer = None

        if self.compute_path_goal_handle is not None:
            self.compute_path_goal_handle.cancel_goal_async()
            self.compute_path_goal_handle = None

        if self.follow_path_goal_handle is not None:
            self.follow_path_goal_handle.cancel_goal_async()
            self.follow_path_goal_handle = None

        self.state = 'IDLE'


    def start_navigation(self):
        self.state = 'PLANNING'
        self.plan_and_navigate()

        # Timer de replanificacion periodica (se re-dispara solo cada replan_period)
        self.replan_timer = self.create_timer(self.replan_period, self.plan_and_navigate)

    def plan_and_navigate(self):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().error(
                'Navegacion fallida en la etapa de localizacion '
                '(TF map->base_link no disponible).')
            self.cancel_active_navigation()
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.current_goal
        goal_msg.start = robot_pose
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = True

        send_future = self.compute_path_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_compute_path_response)

    def _on_compute_path_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'Navegacion fallida en la etapa de planificacion (goal rechazado).')
            self.cancel_active_navigation()
            return

        self.compute_path_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_compute_path_result)

    def _on_compute_path_result(self, future):
        wrapped_result = future.result()

        if wrapped_result.status == GoalStatus.STATUS_CANCELED:
            return  # lo cancelamos nosotros mismos (meta nueva) -> no es un fallo

        result = wrapped_result.result
        if result.error_code != ComputePathToPose.Result.NONE:
            self.get_logger().error(
                f'Navegacion fallida en la etapa de planificacion: {result.error_msg}')
            self.cancel_active_navigation()
            return

        self.compute_path_goal_handle = None
        self.send_follow_path(result.path)

    def send_follow_path(self, path):
        self.follow_path_generation += 1
        my_generation = self.follow_path_generation

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id

        send_future = self.follow_path_client.send_goal_async(
            goal_msg, feedback_callback=self._on_follow_path_feedback)
        send_future.add_done_callback(
            lambda f: self._on_follow_path_response(f, my_generation))

    def _on_follow_path_response(self, future, generation):
        if generation != self.follow_path_generation:
            return  # ya se mando un replan mas nuevo, este quedo obsoleto

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'Navegacion fallida en la etapa de control (goal rechazado).')
            self.cancel_active_navigation()
            return

        self.follow_path_goal_handle = goal_handle
        self.state = 'NAVIGATING'

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_follow_path_result(f, generation))

    def _on_follow_path_feedback(self, feedback_msg):
        self.get_logger().info(
            f'Navegando... distancia a la meta: '
            f'{feedback_msg.feedback.distance_to_goal:.2f} m',
            throttle_duration_sec=2.0)

    def _on_follow_path_result(self, future, generation):
        if generation != self.follow_path_generation:
            return  # resultado de un goal viejo, ya reemplazado -> ignorar

        wrapped_result = future.result()
        if wrapped_result.status == GoalStatus.STATUS_CANCELED:
            return  # lo cancelamos nosotros mismos -> no es un fallo

        result = wrapped_result.result
        if result.error_code == FollowPath.Result.NONE:
            self.get_logger().info('Meta alcanzada con exito.')
        else:
            self.get_logger().error(
                f'Navegacion fallida en la etapa de control: {result.error_msg}')

        self.cancel_active_navigation()


def main():
    rclpy.init()
    node = GoalOrchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()