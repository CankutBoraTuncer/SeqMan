import robotic as ry
import numpy as np

class Node:
    main_goal = ""

    def __init__(self, C:ry.Config, g:list, path:list=[], layer_no:int=0, agent_name:str="ego", score:int=-1):
        self.C     = C                         # Configuration
        self.g     = g                         # Goal
        self.o     = g[0]                      # Object
        self.og    = g[1]                      # Object goal
        self.t     = 0                         # How many times this node is tried
        self.agent = agent_name                # Agent name
        self.path  = path
        self.layer_no = layer_no
        self.score = score
        
    def __repr__(self):
        return f"Object:{self.o} | Times:{self.t} | Agent:{self.agent} | Layer No:{self.layer_no} | Goal:{self.og} | Score:{self.score}"

    def __str__(self):
        return f"Object:{self.o} | Times:{self.t} | Agent:{self.agent} | Layer No:{self.layer_no} | Goal:{self.og} | Score:{self.score}"