from utils.Point import Point

class ObstacleManager:
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self, point: Point):
        """Adiciona um obstáculo à lista."""
        print(f" obstaculo: {point.x}, {point.y}")
        self.obstacles.append(point)

    def remove_obstacle(self, point: Point):
        """Remove um obstáculo da lista."""
        self.obstacles.remove(point)

    def get_obstacles(self):
        """Retorna a lista de obstáculos."""
        return self.obstacles

    def clear_obstacles(self):
        """Limpa todos os obstáculos."""
        self.obstacles.clear()

