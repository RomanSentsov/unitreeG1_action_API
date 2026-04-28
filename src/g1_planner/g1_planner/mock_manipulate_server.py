import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from robot_interfaces.action import Manipulate
import time

class MockManipulateServer(Node):
    def __init__(self):
        super().__init__('mock_manipulate_server')
        self._server = ActionServer(self, Manipulate, 'manipulate', self.execute_callback)
        self.get_logger().info('Mock-сервер манипуляции запущен')

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'Получена цель: object={goal.object_id}, action={goal.action}')
        
        if goal.object_id == 'button' and goal.action != 'press':
            goal_handle.abort()
            result = Manipulate.Result()
            result.success = False
            result.message = f'Действие {goal.action} не поддерживается для {goal.object_id}'
            result.error_code = 'INVALID_ACTION'
            return result

        feedback = Manipulate.Feedback()
        for step in range(1, 5):
            feedback.status = f'Выполнение {goal.action} шаг {step}/4'
            feedback.progress = step / 4.0
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(feedback.status)
            time.sleep(1)

        goal_handle.succeed()
        result = Manipulate.Result()
        result.success = True
        result.message = f'{goal.action} над {goal.object_id} выполнено'
        result.error_code = 'SUCCESS'
        return result

def main(args=None):
    rclpy.init(args=args)
    server = MockManipulateServer()
    rclpy.spin(server)

if __name__ == '__main__':
    main()
