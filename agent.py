from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from planner import Planner
from math import sqrt

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, planner=None):
        super().__init__(id, yellow)
        self.path = []  # Lista para armazenar o caminho a ser seguido
        self.planner = planner

    def decision(self):
        if len(self.targets) == 0:
            return

        # Caso o caminho ainda não tenha sido calculado
        if not self.path:
            # Calcula o caminho usando A* com base no alvo atual
            sx, sy = self.robot.x, self.robot.y  # Posição inicial do robô
            gx, gy = self.targets[0].x, self.targets[0].y  # Posição do alvo
            
            print(f" posição target: {gx}, {gy}")
            print(f" posição robot: {sx}, {sy}")

            self.path = self.planner.planning(sx, sy, gx, gy)
            # Aqui você chama o método planning para calcular o caminho
            print(f'path: {self.path}') #está retornando none

        # Se houver um caminho calculado, o robô vai em direção ao próximo ponto
        if self.path and len(self.path[0]) > 0 and len(self.path[1]) > 0 :
            next_posicao = Point(self.path[0][0], self.path[1][0])  # Pega o primeiro ponto do caminho
            
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_posicao)
            self.set_vel(target_velocity)  # Define a velocidade do robô
            self.set_angle_vel(target_angle_velocity)  # Define a rotação do robô

            distance_to_target = sqrt((self.robot.x - next_posicao.x)**2 + (self.robot.y - next_posicao.y)**2)

            if distance_to_target < 0.2:  # Ajuste o valor de tolerância para o seu caso
                self.path[0].pop(0)  # Remove a primeira coordenada x
                self.path[1].pop(0)  # Remove a primeira coordenada y

                # Se o caminho está vazio, o robô alcançou o alvo atual
                if len(self.path[0]) == 0 or len(self.path[1]) == 0:
                    self.targets.pop(0) # Remove o alvo atingido
                    self.path = []  # Reseta o caminho para calcular o próximo alvo
        
        return


    def post_decision(self):
        pass
