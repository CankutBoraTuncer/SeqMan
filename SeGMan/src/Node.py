import robotic as ry

class Node():
    def __init__(self, C:ry.Config, op:list=None, layer:int=0):
        self.C = ry.Config()
        self.C.addConfigurationCopy(C)
        self.op = op
        self.layer = layer
        self.visit = 0
        self.FS = []
        self.score = 0
        self.heatmap = {}

