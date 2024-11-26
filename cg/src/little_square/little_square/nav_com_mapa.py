#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cg_interfaces.srv import GetMap, MoveCmd
import numpy as np
from queue import PriorityQueue

class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')
        self.move_client = self.create_client(MoveCmd, '/move_command')
        self.map_client = self.create_client(GetMap, '/get_map')
        while not self.move_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Esperando pelo serviço /move_command...')
        while not self.map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Esperando pelo serviço /get_map...')
        self.get_logger().info('Conectado aos serviços!')

    def get_map(self):
        """Obtém o mapa do serviço /get_map."""
        req = GetMap.Request()
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        grid_flat = response.occupancy_grid_flattened
        shape = response.occupancy_grid_shape
        grid = np.array(grid_flat).reshape(shape)  # Reconstrói o grid 2D
        return grid

    def move_robot(self, direction):
        """Envia o comando de movimento para o robô."""
        req = MoveCmd.Request()
        req.direction = direction
        future = self.move_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def a_star_path(self, grid, start, goal):
        """Planeja a rota do robô usando o algoritmo A*."""
        rows, cols = grid.shape
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                break

            for d in directions:
                next_node = (current[0] + d[0], current[1] + d[1])
                if 0 <= next_node[0] < rows and 0 <= next_node[1] < cols:
                    if grid[next_node[0]][next_node[1]] != 'b':  # Ignorar bloqueios
                        new_cost = cost_so_far[current] + 1
                        if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                            cost_so_far[next_node] = new_cost
                            priority = new_cost + heuristic(goal, next_node)
                            open_set.put((priority, next_node))
                            came_from[next_node] = current

        # Reconstruir o caminho
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def follow_path(self, path):
        """Segue o caminho planejado movendo o robô."""
        directions_map = {
            (0, -1): "left",
            (0, 1): "right",
            (-1, 0): "up",
            (1, 0): "down"
        }
        for i in range(len(path) - 1):
            move = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            direction = directions_map[move]
            response = self.move_robot(direction)
            if not response.success:
                self.get_logger().error(f"Movimento {direction} falhou!")

    def navigate(self):
        """Controla o fluxo completo de navegação."""
        grid = self.get_map()
        self.get_logger().info(f"Mapa recebido:\n{grid}")
        start = tuple(np.argwhere(grid == 'r')[0])  # Encontra o robô
        goal = tuple(np.argwhere(grid == 't')[0])  # Encontra o alvo
        self.get_logger().info(f"Posição inicial: {start}, Alvo: {goal}")

        path = self.a_star_path(grid, start, goal)
        self.get_logger().info(f"Caminho planejado: {path}")
        self.follow_path(path)
        self.get_logger().info("Navegação concluída!")

def main(args=None):
    rclpy.init(args=args)
    navigator = MapNavigator()
    navigator.navigate()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
