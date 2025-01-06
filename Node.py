import robotic as ry
import numpy as np

from utils import (sample_uniform_points, compute_heuristic, find_path_between_configurations, 
                   move_on_path)

class Node:
    def __init__(self, C, level, id, type, EGO_NAME, OBJ_NAME, parentId = None):
        self.config = ry.Config()
        self.config.addConfigurationCopy(C) 
        self.parentId = parentId
        self.id = id
        self.level = level
        self.type = type # type = "pick" or "place"
        self.EGO_NAME = EGO_NAME
        self.OBJ_NAME = OBJ_NAME
        if self.type == "place":
            self.config.attach(EGO_NAME, OBJ_NAME)

    def branch_from_place_node(self, id, GOAL_NAME):
        sampled_points = sample_uniform_points(self.config, num_samples= 300)
        level = self.level
        scores = []
        for point in sampled_points:
            score = compute_heuristic(self.config, point, self.EGO_NAME, GOAL_NAME)
            scores.append((point, score))
        scores.sort(key=lambda x: x[1], reverse=True)

        path = []
        for idx in range(len(scores)):
            path = find_path_between_configurations(self.config, self.config.getJointState(), scores[idx][0])
            if isinstance(path, np.ndarray):
                if path.ndim < 1:  
                    print("empty path")
                    continue
                else:
                    found = True
                    break
        if found:
            converged = move_on_path(self.config, path)
            self.config.frame(self.OBJ_NAME).unLink()
            newNode = Node(self.config, self.level + 1, id, "pick", self.EGO_NAME, self.OBJ_NAME, self.id)
            return newNode
        return None
    
    def __repr__(self):  
        return "| Type: % s, Level: % s, ID: % s, ParentID: % s |" % (self.type, self.level, self.id, self.parentId)
    
    def __str__(self):  
        return "| Type: % s, Level: % s, ID: % s, ParentID: % s |" % (self.type, self.level, self.id, self.parentId)