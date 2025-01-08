import robotic as ry
import numpy as np

class Node:
    main_goal = ""

    def __init__(self, C:ry.Config, g:list, path:list=[], agent_name:str="ego"):
        self.C     = C                         # Configuration
        self.g     = g                         # Goal
        self.o     = g[0]                      # Object
        self.og    = g[1]                      # Object goal
        self.t     = 0                         # How many times this node is tried
        self.agent = agent_name                # Agent name
        self.path  = path
        
    def __repr__(self):
        return f"Node({self.o}, {self.og}, {self.t}, {self.agent}, {self.path})"

    def __str__(self):
        return f"Node({self.o}, {self.og}, {self.t}, {self.agent}, {self.path})"