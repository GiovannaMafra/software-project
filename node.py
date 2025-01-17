class Node:
    def __init__(self, x, y, cost, f, parent_index):
        self.x = x  #índice da grade
        self.y = y  #índice da grade
        self.cost = cost #valor de g
        self.f = f #coloquei f como atributo para poder comparar o de todos com o de todos
        self.parent_index = parent_index  #índice do nó anterior
             

