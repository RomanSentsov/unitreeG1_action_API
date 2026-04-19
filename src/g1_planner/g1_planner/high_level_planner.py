import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_interfaces.action import Manipulate
import json
import sys

class HighLevelPlanner(Node):
    def __init__(self):
        super().__init__('high_level_planner')
        self.get_logger().info('Планировщик запущен')
        with open('world_model.json', 'r') as f:
            self.world = json.load(f)

    def validate_manipulate(self, object_id, action):
        obj = next((o for o in self.world['objects'] if o['id'] == object_id), None)
        if not obj:
            return False, f'Объект "{object_id}" не найден'
        if action not in obj['actions']:
            return False, f'Действие "{action}" не разрешено'
        return True, 'OK'

    def run_manipulate(self, object_id, action):
        valid, msg = self.validate_manipulate(object_id, action)
        if not valid:
            self.get_logger().error(msg)
            return

        client = ActionClient(self, Manipulate, 'manipulate')
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Сервер манипуляции не отвечает')
            return

        goal = Manipulate.Goal()
        goal.object_id = object_id
        goal.action = action

        self.get_logger().info(f'Отправка goal: object={object_id}, action={action}')
        send_future = client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Успех: {result.message}')
        else:
            self.get_logger().error(f'Ошибка: {result.message} (код: {result.error_code})')

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {fb.status} (прогресс: {fb.progress*100:.0f}%)')

    def run_script(self, script_file):
        with open(script_file, 'r') as f:
            lines = f.readlines()
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if not parts:
                continue
            cmd = parts[0].lower()
            if cmd == 'manipulate' and len(parts) >= 3:
                object_id = parts[1]
                action = parts[2]
                self.run_manipulate(object_id, action)
            else:
                self.get_logger().error(f'Неизвестная команда: {line}')

def main(args=None):
    rclpy.init(args=args)
    planner = HighLevelPlanner()
    
    if len(sys.argv) > 1:
        if sys.argv[1] == 'manipulate' and len(sys.argv) >= 4:
            object_id = sys.argv[2]
            action = sys.argv[3]
            planner.run_manipulate(object_id, action)
        else:
            print('Использование: python3 high_level_planner.py manipulate <object> <action>')
    else:
        planner.run_script('script.txt')
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
