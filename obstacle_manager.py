from utils.Point import Point

class ObstacleManager:
    def __init__(self):
        self.obstacles = []

    def add_obstacle(self, point: Point):
        #adiciona um obstáculo à lista
        #print(f" obstaculo: {point.x}, {point.y}")
        self.obstacles.append(point)

    def get_obstacles(self):
        #retorna a lista de obstáculos
        return self.obstacles

