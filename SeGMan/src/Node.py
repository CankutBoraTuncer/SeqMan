import robotic as ry


class Node():
    def __init__(self, C:ry.Config, pair=None, parent:"Node"=None, layer:int=0, FS:list=[], total_score:float=0.0, prev_scene_scores:dict={}, init_scene_scores:dict={}):
        self.C = ry.Config()
        self.C.addConfigurationCopy(C)
        self.pair = pair
        self.layer = layer
        self.visit = 1
        self.FS = FS
        self.total_score = total_score
        self.prev_scene_scores = prev_scene_scores
        self.init_scene_scores = init_scene_scores
        self.parent = parent

    def __str__(self):
        return "Node: layer={}, visit={}, pair={}, total_score={}".format(self.layer, self.visit, self.pair, self.total_score)
    
    def __repr__(self):
        return self.__str__()

