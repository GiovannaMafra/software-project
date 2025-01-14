from planner import Planner
from obstacle_manager import ObstacleManager

resolution = 0.1
min_x = -3.0
max_x = 3.0
min_y = -2.0
max_y = 2.0
obstacle_manager = ObstacleManager()

planner = Planner(resolution, min_x, max_x, min_y, max_y, obstacle_manager)
print("Planner criado com sucesso:", planner)
