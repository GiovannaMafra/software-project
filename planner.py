from node import Node 
from obstacle_manager import ObstacleManager 
from math import sqrt

class Planner: 
    def __init__(self, resolution, min_x, max_x, min_y, max_y, obstacle_manager): 
    # Inicialização de variáveis e mapas pass
        self.resolution = resolution
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.obstacle_manager = obstacle_manager 
        #self.motion = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, sqrt(2)), (1, -1, sqrt(2)), (-1, 1, sqrt(2)), (-1, -1, sqrt(2))]
        #self.motion = [(0.5, 0, 0.5), (-0.5, 0, 0.5), (0, 0.5, 0.5), (0, -0.5, 0.5), (0.5, 0.5, sqrt(0.5)*0.5), (0.5, -0.5, sqrt(0.5)*0.5), (-0.5, 0.5, sqrt(0.5)*0.5), (-0.5, -0.5, sqrt(0.5)*0.5)]
        self.motion = [(0.5, 0, 0.5), (-0.5, 0, 0.5), (0, 0.5, 0.5), (0, -0.5, 0.5)]

    def planning(self, sx, sy, gx, gy):
 
  #Pesquisa de caminho usando A*


        #entrada:
            #sx: posição inicial x [m]
            #sy: posição inicial y [m]
            #gx: posição final x [m]
            #gy: posição final y [m]


        #saída:
            #rx: posição final x lista do caminho final
            #ry: posição final y lista do caminho final


        start_node = Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, 0, -1)
        goal_node = Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, 0, -1)
    
        #Initialize the open list
        #Initialize the closed list        
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node #adicionar o f = 0
        obstacles = set((obs.x, obs.y) for obs in self.obstacle_manager.get_obstacles())


        while(len(open_set) > 0):
            #achar o nó na open_set com menor f, call it "q"
            q = min(open_set, key=lambda o: open_set[o].f)
            current = open_set[q]
            # Remove o item no conjunto aberto
            del open_set[q]

            #se o node atual é o final, acaba
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                #rx, ry = self.reconstruct_path(goal_node, closed_set)
                #print(f"Expansão do nó: ({rx}, {ry})")
                return self.reconstruct_path(goal_node, closed_set)


            # Adiciona ao conjunto fechado
            closed_set[q] = current


            #gera os 8 possiveis movimentos do robot (4 diag + frente + atrás, esquerda, direita)
            for move_x, move_y, move_cost in self.motion:


                #successor.h = distance from goal to successor
                h = sqrt ((current.x + move_x - goal_node.x)**2 + (current.y + move_y - goal_node.y)**2 )


                #f = g + h
                f =  current.cost + move_cost + 1.5*h

                node = Node(current.x + move_x, current.y + move_y, current.cost + move_cost, f,  q)
                n_id = self.calc_index(node)


                if not self.verify_node(node, obstacles):
                    continue


                #if a node with the same position as successor is in the OPEN list which has a lower f than successor, skip this successor
                if n_id in open_set and open_set[n_id].f < node.f:
                    continue


                #if a node with the same position as successor  is in the CLOSED list which has a lower f than successor, skip this successor otherwise, add  the node to the open list
                if n_id in closed_set and closed_set[n_id].f < node.f:
                    continue
            
                open_set[n_id] = node



    def calc_xy_index(self, position, minp):
            return round((position - minp) / self.resolution)


    def calc_index(self, node):
            return (node.y - self.min_y) * 6 + (node.x - self.min_x)
            #return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)


    def calc_position(self, index, minp):
        return index * self.resolution + minp 
        #essa resolution deve estar errada
        #pensei que fosse como uma grid, por isso coloquei as moviemntações do motion
        #mas eu não sei como funciona com as celulas


    def verify_node(self, node, obstacles):
            px = self.calc_position(node.x, self.min_x)
            py = self.calc_position(node.y, self.min_y)

            #print(f"Expansão do nó: ({px}, {py})")
            if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
                #print('oi')
                return False
            
            #for obs in obstacles:
                #obs_px = self.calc_position(obs[0], self.min_x)
                #obs_py = self.calc_position(obs[1], self.min_y)
                #distance_to_obstacle = sqrt((px - obs_px)**2 + (py - obs_py)**2)
                #if distance_to_obstacle <= safety_radius:
                    #return False
            safety_radius = 0.3 # Raio de segurança em unidades do mapa
            for obs in obstacles:
                distance_to_obstacle = ((node.x - obs[0])**2 + (node.y - obs[1])**2)** 0.5
                if distance_to_obstacle <= safety_radius:
                    return False

            return True


    def reconstruct_path(self, goal_node, closed_set):
        rx, ry = [], []
        current_node = goal_node

        while current_node.parent_index != -1:
            # Converter de índices de grade para coordenadas reais
            real_x = self.calc_position(current_node.x, self.min_x)
            real_y = self.calc_position(current_node.y, self.min_y)
            rx.append(real_x)
            ry.append(real_y)

            # Mover para o nó pai
            current_node = closed_set[current_node.parent_index]

        # Adicionar a posição inicial (start_node) no final do loop
        real_x = self.calc_position(current_node.x, self.min_x)
        real_y = self.calc_position(current_node.y, self.min_y)
        rx.append(real_x)
        ry.append(real_y)

        # Inverter para que o caminho esteja na ordem correta (start -> goal)
        return [rx[::-1], ry[::-1]]





