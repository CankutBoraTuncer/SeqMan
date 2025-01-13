import robotic as ry
import numpy as np

from utils_old import (find_path_between_configurations, 
                   move_on_path, ik_for_agent_to_object, 
                   solve_ik_for_all_points, filter_solutions_for_agent_to_object, reachable, move_agent_away_from_object, object_faces_goal)

from Node import Node

C = ry.Config()
task = "p8-corner.g"
C.addFile(task)
EGO_NAME = "ego"
OBJ_NAME = "obj"
GOAL_NAME = "goal_visible"

ry.params_add({
    "rrt/stepsize": 0.01,
    "rrt/verbose": -1
})

allNodes = {}
allConfigs = {}
id = 0
node = Node(C, 0, id, "pick", EGO_NAME, OBJ_NAME, parentId=-1) # parentId = -1 ==> root node
allNodes[id] = node
allConfigs[id] = node.config
id +=1
path_nodes = []
L = [node]
node = None

while len(L) > 0:    
    node = L.pop(0)

    print(f"################################    PROCESSING NODE {node.id}, level: {node.level}   ###############################################")
    print(f"Node type: {node.type}")
    print(f"Parent of current node: {node.parentId}")
    print(f"Level of current node: {node.level}")
    #node.config.view(True)
    #node.config.view_close()
    if node.type == "place" and reachable(node.config, node.config.frame(GOAL_NAME)) and object_faces_goal(node.config, node.config.frame(GOAL_NAME)):

        found = False
        while not found:
            path = find_path_between_configurations(node.config, node.config.getJointState(), node.config.frame(GOAL_NAME).getPosition()[0:2])
            if isinstance(path, np.ndarray):
                if path.ndim >= 1:  
                    found = True             
        print("******************************* GOAL REACHABLE ***************************************")

        node.config.view(True)
        node.config.view_close()
        move_on_path(node.config, path, found=True)
        break
    else:
        if node.type == "pick":
            solutions = solve_ik_for_all_points(node.config, EGO_NAME, OBJ_NAME)
   
            if len(solutions) == 0:
                print("No solutions found")
                continue

            paths = []
            for i in range(len(solutions)):
                path = find_path_between_configurations(node.config, node.config.getJointState(), solutions[i][-1])
                print(f"Path {i}: {path}")
                if isinstance(path, np.ndarray):
                    if path.ndim < 1:  
                        print("empty path")
                        continue
                paths.append(path)

            for path in paths:
                initialJointState = node.config.getJointState()
                converged = move_on_path(node.config, path)
                newNode = Node(node.config, node.level + 1, id, type="place", EGO_NAME=EGO_NAME, OBJ_NAME=OBJ_NAME, parentId= node.id)
                print(f"added place node with id: {newNode.id}")
                allNodes[id] = newNode
                #newNode.config.view(True)
                id += 1
                node.config.setJointState(initialJointState)
                L.append(newNode)
            
            print(L)

        elif  node.type == "place":            
            node_place = node.branch_from_place_node(id, GOAL_NAME)  
            if node_place is not None:
                L.append(node_place)
                #node_place.config.view(True)
                allNodes[id] = node_place

                id +=1 

print("############################# END OF THE LOOP ################################")


for node in allNodes:
    del node

for node in L:
    del node





