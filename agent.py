from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from planner import Planner
from math import sqrt

class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, planner=None):
        super().__init__(id, yellow)
        self.path = []  #lista para armazenar o caminho a ser seguido
        self.planner = planner
        self.ultima_posicao = None
        self.distancia_preso = 0.0005 #teste
        self.count = 0

    def decision(self):
        if len(self.targets) == 0:
            return
        
        #verifica se o robô está "preso"
        if self.ultima_posicao:
            distance_movida = ((self.robot.x - self.ultima_posicao[0]) ** 2 + (self.robot.y - self.ultima_posicao[1]) ** 2)**0.5
            
            if distance_movida < self.distancia_preso:
                self.count += 1
                if self.count > 4:  #se o robo ficar se movendo muito pouco por mais de 4 interações, está preso
                    self.replanejar()  #recalcula o caminho
            else:
                self.count = 0 #ele conseguiu recalcular uma rota válida

        #Atualiza a última posição do robô
        self.ultima_posicao = (self.robot.x, self.robot.y)

        #caso o caminho ainda não tenha sido calculado
        if not self.path:
            #calcula o caminho usando A* com base no alvo atual
            sx, sy = self.robot.x, self.robot.y  #posiçao inicial do robô
            gx, gy = self.targets[0].x, self.targets[0].y  #posiçao do alvo
            
            #print(f" posição target: {gx}, {gy}")
            #print(f" posição robot: {sx}, {sy}")

            self.path = self.planner.planning(sx, sy, gx, gy)
            #print("recalculou") 

        #se existir um caminho calculado, o robô vai em direção ao próximo ponto
        if self.path and len(self.path[0]) > 0 and len(self.path[1]) > 0 :
            next_posicao = Point(self.path[0][0], self.path[1][0])  #pega o primeiro ponto do caminho
            
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_posicao)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity) 

            distancia_do_target = ((self.robot.x - next_posicao.x)**2 + (self.robot.y - next_posicao.y)**2)**0.5

            if distancia_do_target < 0.2:  #valor de tolerância (não vai chegar na posição exata)
                self.path[0].pop(0)
                self.path[1].pop(0)  

                #se o caminho está vazio, o robô alcançou o alvo atual
                if len(self.path[0]) == 0 or len(self.path[1]) == 0:
                    self.targets.pop(0) #remove alvo atingido
                    self.path = []  #reseta o caminho
        
        return


    def post_decision(self):
        pass

    def replanejar(self):
        #recalcula o caminho
        sx, sy = self.robot.x, self.robot.y
        gx, gy = self.targets[0].x, self.targets[0].y
        self.path = self.planner.planning(sx, sy, gx, gy)
