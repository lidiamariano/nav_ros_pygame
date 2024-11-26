#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cg_interfaces.srv import MoveCmd
import numpy as np

class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')
        self.client = self.create_client(MoveCmd, '/move_command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando pelo serviço /move_command...')
        self.get_logger().info('Conectado ao serviço /move_command!')

    def move_robot(self, direction):
        """Envia o comando de movimento para o robô."""
        req = MoveCmd.Request()
        req.direction = direction
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Movendo para direção:'+ req.direction + '!' )
        return future.result()

    def get_next_position(self, current_pos, direction):
        """Calcula a próxima posição com base na direção."""
        x, y = current_pos
        if direction == "left":
            return x, y - 1
        elif direction == "right":
            return x, y + 1
        elif direction == "up":
            return x - 1, y
        elif direction == "down":
            return x + 1, y

    def navigate(self):
        """Controla o fluxo de navegação reativa."""
        visited = {}  # Dicionário para armazenar posições visitadas e suas contagens
        while True:
            response = self.move_robot("")  # Chamada sem direção para obter o estado atual
            robot_pos = tuple(response.robot_pos)
            target_pos = tuple(response.target_pos)

            if robot_pos == target_pos:
                self.get_logger().info("Alvo alcançado!")
                break

            visited[robot_pos] = visited.get(robot_pos, 0) + 1

            # Escolher direção baseada nos sensores e histórico de visitas
            directions = {
                "down": response.down,
                "right": response.right,
                "left": response.left,
                "up": response.up,
            }

            possible_moves = []
            for direction, status in directions.items():
                next_pos = self.get_next_position(robot_pos, direction)
                if status == 't':  # Alvo encontrado
                    self.move_robot(direction)
                    return
                elif status == 'f' and next_pos not in visited:  # Posição livre e não visitada
                    possible_moves.append((direction, 0))  # Adiciona com prioridade alta
                elif status == 'f':
                    possible_moves.append((direction, visited[next_pos]))  # Adiciona com contagem de visitas

            if possible_moves:
                # Escolher o movimento com a menor contagem de visitas
                move = min(possible_moves, key=lambda x: x[1])[0]
                self.move_robot(move)

def main(args=None):
    rclpy.init(args=args)
    navigator = ReactiveNavigator()
    navigator.navigate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
