from node import Node 
from obstacle_manager import ObstacleManager 
from math import sqrt

class Planner: 
    def __init__(self, resolution, min_x, max_x, min_y, max_y, obstacle_manager): 
        self.resolution = resolution
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.obstacle_manager = obstacle_manager 
        #testando qual movimentação seria melhor
        #self.motion = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, sqrt(2)), (1, -1, sqrt(2)), (-1, 1, sqrt(2)), (-1, -1, sqrt(2))]
        #self.motion = [(0.5, 0, 0.5), (-0.5, 0, 0.5), (0, 0.5, 0.5), (0, -0.5, 0.5), (0.5, 0.5, sqrt(0.5)*0.5), (0.5, -0.5, sqrt(0.5)*0.5), (-0.5, 0.5, sqrt(0.5)*0.5), (-0.5, -0.5, sqrt(0.5)*0.5)]
        #self.motion = [(0.5, 0, 0.5), (-0.5, 0, 0.5), (0, 0.5, 0.5), (0, -0.5, 0.5)]
        self.motion = [(0.3, 0, 0.3), (-0.3, 0, 0.3), (0, 0.3, 0.3), (0, -0.3, 0.3), (0.3, 0.3, sqrt(0.3)*0.3), (0.3, -0.3, sqrt(0.3)*0.3), (-0.3, 0.3, sqrt(0.3)*0.3), (-0.3, -0.3, sqrt(0.3)*0.3)]

    def planning(self, sx, sy, gx, gy):
 
  #planejamento de caminho usando A*

        start_node = Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, 0, -1)
        goal_node = Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, 0, -1)
          
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node
        obstacles = set((obs.x, obs.y) for obs in self.obstacle_manager.get_obstacles())

        path_steps = 0  #contador de passos 

        while(len(open_set) > 0):
            #achar o nó na open_set com menor f 
            q = min(open_set, key=lambda o: open_set[o].f)
            current = open_set[q]
            #remove do conjunto aberto
            del open_set[q]

            path_steps += 1
            #se o node atual é o final, acaba // se o node for maior que os steps
            if path_steps >= 5 or (current.x == goal_node.x and current.y == goal_node.y):
                return self.reconstroi_path(current, closed_set)

            closed_set[q] = current

            #gera os 8 possiveis movimentos do robot (4 diag + frente + atrás, esquerda, direita)
            #de acordo com a resolution/tamanho das celulas da grid
            for move_x, move_y, move_cost in self.motion:

                #calcula o custo baseado se existe ou não um obstaculo perto, se existir o custo aumenta uma penalidade
                current.cost = self.calcula_custo(current, obstacles)

                #heuristica euclidiana
                h = ((current.x + move_x - goal_node.x)**2 + (current.y + move_y - goal_node.y)**2 )**0.5

                #f = g + h
                f =  current.cost + move_cost + h

                node = Node(current.x + move_x, current.y + move_y, current.cost + move_cost, f,  q)
                #ajuusta o index da grid
                n_id = self.calc_index(node)

                if not self.verifica_node(node, n_id, closed_set):
                    continue

                #se existe a posicao na open list, mas com um f menor, ignora essa
                if n_id in open_set and open_set[n_id].f < node.f:
                    continue

                #se existe a posicao na closed list, mas com f menor, ignora
                if n_id in closed_set and closed_set[n_id].f < node.f:
                    continue
            
                open_set[n_id] = node



    def calc_xy_index(self, position, min):
            return round((position - min) / self.resolution)


    def calc_index(self, node):
            return (node.y - self.min_y) * 6 + (node.x - self.min_x)
            #return (node.y - self.min_y) * largura_x + (node.x - self.min_x)


    def calc_posicao(self, index, min):
        return index * self.resolution + min
        #repensar resolution

    def calcula_custo(self, current, obstacles):
        cost = current.cost
        raio_seguranca = 0.3 
        for obs in obstacles:
            distance_to_obstacle = ((current.x - obs[0])**2 + (current.y - obs[1])**2)** 0.5
            if distance_to_obstacle <= raio_seguranca:
                cost += 100 #penalidade
        return cost

    def verifica_node(self, node, n_id, closed_set):
            px = self.calc_posicao(node.x, self.min_x)
            py = self.calc_posicao(node.y, self.min_y)

            if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
                return False
            
            #tentar não voltar posiçoes
            if  n_id in closed_set:
                return False
            
            return True

    def reconstroi_path(self, goal_node, closed_set):
        rx, ry = [], []
        current_node = goal_node

        while current_node.parent_index != -1:
            #converte de índices da grid para coordenadas reais
            real_x = self.calc_posicao(current_node.x, self.min_x)
            real_y = self.calc_posicao(current_node.y, self.min_y)
            rx.append(real_x)
            ry.append(real_y)

            #move para o nó pai
            current_node = closed_set[current_node.parent_index]

        #adiciona a posicao inicial no final do loop, para quando inverter ficar certo
        real_x = self.calc_posicao(current_node.x, self.min_x)
        real_y = self.calc_posicao(current_node.y, self.min_y)
        rx.append(real_x)
        ry.append(real_y)

        #inverte para que o caminho fique na ordem certa (inicio -> goal)
        return [rx[::-1], ry[::-1]]





