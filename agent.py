from utils.ssl.Navigation import Navigation
from utils.ssl.base_agent import BaseAgent
from utils.Point import Point
from planner import Planner


class ExampleAgent(BaseAgent):
    def __init__(self, id=0, yellow=False, planner=None):
        super().__init__(id, yellow)
        self.path = []  #lista para armazenar o caminho a ser seguido
        self.planner = planner
        self.ultima_posicao = None
        self.count = 0
        self.visitados = set()

    def decision(self):
        if len(self.targets) == 0:
            return
       
        alvo_mais_proximo = None 
        indice_mais_proximo = -1 

        #Atualiza a última posição do robô
        self.ultima_posicao = (self.robot.x, self.robot.y)

        #caso o caminho ainda não tenha sido calculado
        if not self.path:
            if len(self.visitados) > 1:
                self.visitados.pop()

            #parte que divide os robos entre os alvos
            if len(self.targets) > 1:
                distancia_minima = 1000 #numero grande

                for i in range(len(self.targets)):
                    distancia_media = ((self.robot.x - self.targets[i].x)**2 + (self.robot.y - self.targets[i].y)**2)**0.5
                    if distancia_media < distancia_minima:
                        distancia_minima = distancia_media
                        alvo_mais_proximo = self.targets[i]
                        indice_mais_proximo = i
                
            else:
                #se só tem um target recebe o primeiro
                indice_mais_proximo = 0
                alvo_mais_proximo = self.targets[indice_mais_proximo]

            #calcula o caminho usando A* com base no alvo atual
            sx, sy = self.robot.x, self.robot.y  #posiçao inicial do robô
            gx, gy = alvo_mais_proximo.x, alvo_mais_proximo.y  #posiçao do alvo

            self.path = self.planner.planning(sx, sy, gx, gy, self.visitados)

        #se existir um caminho calculado, o robô vai em direção ao próximo ponto
        if self.path and len(self.path[0]) > 0 and len(self.path[1]) > 0 :
            next_posicao = Point(self.path[0][0], self.path[1][0])  #pega o primeiro ponto do caminho
           
            target_velocity, target_angle_velocity = Navigation.goToPoint(self.robot, next_posicao)
            self.set_vel(target_velocity)
            self.set_angle_vel(target_angle_velocity)

            distancia_do_target = ((self.robot.x - next_posicao.x)**2 + (self.robot.y - next_posicao.y)**2)**0.5


            if distancia_do_target < 0.05:  #valor de tolerância (não vai chegar na posição exata)
                self.path[0].pop(0)
                self.path[1].pop(0)  


                #se o caminho está vazio, o robô alcançou o alvo atual
                if len(self.path[0]) == 0 or len(self.path[1]) == 0:
                    self.targets.pop(indice_mais_proximo) #remove alvo atingido
                    #if alvo_mais_proximo in self.targets_ocupados:
                    self.path = []  #reseta o caminho
       
        return

    def post_decision(self):
        pass




