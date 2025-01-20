import robotic as ry

class Node():
    def __init__(self, C:ry.Config, op:list=None, layer:int=0, FS:list=[], score:float=0.0):
        self.C = ry.Config()
        self.C.addConfigurationCopy(C)
        self.op = op
        self.layer = layer
        self.visit = 1
        self.FS = FS
        self.score = score

    def __str__(self):
        return "Node: layer={}, visit={}, op={}, score={}".format(self.layer, self.visit, self.op, self.score)
    
    def __repr__(self):
        return self.__str__()

