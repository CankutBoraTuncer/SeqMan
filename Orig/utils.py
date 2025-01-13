import robotic as ry
import numpy as np
from Node import Node
import time
import os
import sys
from contextlib import contextmanager

@contextmanager
def suppress_stdout():
    """Suppress output from C++ std::cout and std::cerr."""
    # Open /dev/null
    devnull = os.open(os.devnull, os.O_WRONLY)
    # Save the original stdout and stderr file descriptors
    original_stdout = os.dup(1)
    original_stderr = os.dup(2)
    
    try:
        # Redirect stdout and stderr to /dev/null
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        # Restore stdout and stderr to their original file descriptors
        os.dup2(original_stdout, 1)
        os.dup2(original_stderr, 2)
        # Close the duplicate file descriptors
        os.close(devnull)
        os.close(original_stdout)
        os.close(original_stderr)
        
def is_line_of_sight(C, obj, goal, view):
    # Get the object and goal positions and sizes
    obj_pos = C.getFrame(obj).getPosition()
    obj_size = C.getFrame(obj).getSize()[:2]
    goal_pos = C.getFrame(goal).getPosition()
    goal_size = C.getFrame(goal).getSize()[:2]

    start = obj_pos[:2]
    end = goal_pos[:2]
    
    # Calculate the midpoint of the line
    midpoint = (start + end) / 2

    # Calculate the orientation (angle) of the line and the length of line
    delta = end - start
    size = np.linalg.norm(abs(delta) - obj_size/2 - goal_size/2 - [0.05, 0.05]) # the last element is error margin
    angle = np.arctan2(delta[1], delta[0])  # Angle in radians

    # Create a temporary configuration to add the intermediate object
    config = ry.Config()
    config.addConfigurationCopy(C)
    config.getFrame(obj).setContact(0)
    config.getFrame(goal).setContact(0)
    collisions = config.getCollisionsTotalPenetration()

    # Add a new object at the midpoint with the calculated orientation
    intermediate_obj = "intermediate_object"
    config.addFrame(intermediate_obj)\
        .setShape(ry.ST.ssBox,[size, 0.1,  .05, .005]).setPosition(np.array([midpoint[0], midpoint[1], 0.12])) \
        .setContact(1)\
        .setQuaternion([np.cos(angle / 2), 0, 0, np.sin(angle / 2)])  # Rotation around Z-axis
    #config.view(True)

    # Check for collisions
    config1 = ry.Config()
    config1.addConfigurationCopy(config)
    collisions1 = config1.getCollisionsTotalPenetration()

    if view:
        config1.view(True, f"With inter{collisions1}")

    del config
    del config1

    if abs(collisions1 - collisions) > 0.01:  
        return 0

    # If no collisions are detected, there is a line of sight
    return 1


def score_function(x:Node):
    
    goal   = x.g[1] 
    
    # Create a temporary configuration to add the intermediate object
    config = ry.Config()
    config.addConfigurationCopy(x.C)

    # The subgoal scoring heuristic
    vg = is_line_of_sight(config, x.g[0], x.agent, False)  # Check line of sight between object goal and agent

    # the agent is not counted as an obstacle
    config.delFrame(x.agent)
    v0 = is_line_of_sight(config, "goal", x.g[0], False)   # Check line of sight between goal and object goal
    
    
    final_goal_pos = config.getFrame("goal").getPosition()

    # proximity d of object goal to the goal, and is set to 5 if d < 0.2, 2 if d < 0.4, and 0 otherwise.
    d = np.linalg.norm(goal[:2] - final_goal_pos[:2])

    vdist = 0

    if d < 2:
        vdist = 5
    elif d < 4:
        vdist = 2
        
    #print(10*v0 + 5*vg + vdist)  
    return 10*v0 + 5*vg + vdist

def select_node(L:list):                                            # Returns the node with the lowest cost
    min_score = np.inf
    min_node  = None
    t         = 0                                                   # Min try count 

    while t < 2:                                                    # Each node can be tried twice 
        for x in L:        
            if x.t == t:
                score = score_function(x)
                if score  < min_score:                              # Node with the least try count has more priority
                    min_score = score
                    min_node = x

        if min_node is not None:                                    # Leave if a node is found
            break
        t += 1

    if min_node is None:
        return None
    
    min_node.t += 1                                                 # Increment the number of times this node is tried
    return min_node
        
def solve(x:Node, g:list):  
    print("Solving")                                        # Solve the task from the current configuration x to the end goal g
    config = ry.Config()
    config.addConfigurationCopy(x.C)
    agent  = x.agent
    obj    = g[0]
    goal   = g[1]
    config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .05 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition(goal)                        # Add goal frame
    p = x.path[:]
    is_feas = False
    frame_st = None

    S = ry.Skeleton()
    S.enableAccumulatedCollisions(True)

    S.addEntry([0, 0.1], ry.SY.touch, [agent, obj])
    S.addEntry([0.1, 0.25], ry.SY.stable, [agent, obj]) 
    S.addEntry([0.2, -1], ry.SY.above, [obj, "subgoal"])
    S.addEntry([0.25, -1], ry.SY.stableOn, ["floor", obj])

    komo = S.getKomo_path(config, 100, 1e-3, 1e3, 1e-5, 1e3)
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()              # Solve

    #print(ret.eq, ret.ineq, ret.sos, ret.f)
    #r = komo.report(True, True, True)       
    #komo.view_play(True, str(ret.feasible), 0.3)

    if ret.feasible:
        #komo.view_play(True, str(ret.feasible), 0.3)
        komo_path = komo.getPath()
        q1 = config.getJointState()
        q2 = komo_path[10]
        q3 = komo_path[-2]

        config1 = ry.Config()
        config1.addConfigurationCopy(config)

        with suppress_stdout():
            rrt1 = ry.PathFinder()                                           # Solve Bi-Directional RRT
            rrt1.setProblem(config1, [q1], [q2])
            solution1 = rrt1.solve()

        if solution1.feasible:

            config2 = ry.Config()
            config2.addConfigurationCopy(config1)
            
            config2.setJointState(solution1.x[-1])
            config2.attach(agent, obj)
            
            with suppress_stdout():
                rrt2 = ry.PathFinder()                                           # Solve Bi-Directional RRT
                rrt2.setProblem(config2, [q2], [q3])
                solution2 = rrt2.solve()

            if solution2.feasible:
                p.append(solution1.x)
                p.append(solution2.x)
                config2.setJointState(solution2.x[-1])
                config2.frame(obj).unLink()
                is_feas = True
                frame_st = config2.getFrameState()

    config.delFrame("subgoal")

    if is_feas:
        config.setFrameState(frame_st)
        return Node(config, [obj, goal], path=p), is_feas
    else:
        return x, is_feas
    

def reachable(x:Node, o:str):                                       # Return True if the agent can reach the object
    config       = ry.Config()
    config.addConfigurationCopy(x.C)

    q_agent  = config.getJointState()
    q_goal   = config.frame(o).getPosition()[0:2]
    obj = config.getFrame(o)
    del obj
    
    with suppress_stdout():
        rrt = ry.PathFinder()                                           # Solve Bi-Directional RRT
        rrt.setProblem(config, [q_agent], [q_goal], collisionTolerance=0.01)
        solution = rrt.solve()
    #for js in solution.x:
    #    config.setJointState(js)
    #    config.view(False, "RRT")
    #    time.sleep(0.01)

    return solution.feasible

def reachable_together(x:Node):                                       # Return True if the agent can reach the object
    config       = ry.Config()
    config.addConfigurationCopy(x.C)
    o = x.o
    pn = x.og[:2]
    obj = config.getFrame(o)
    ag  = config.getFrame(x.agent)
    q_agent  = config.getJointState()
    
    
    if obj.getParent() != x.agent:
        config.view(True, "not attach")
        q_goal   = pn
        del obj

    else:
        config.view(True, "attach")
        pp = obj.getPosition()[0:2]
        pd = pn - pp
        q_agent = config.getJointState()+pd


    rrt = ry.PathFinder()                                           # Solve Bi-Directional RRT
    rrt.setProblem(config, [q_agent], [q_goal], collisionTolerance=0.01)
    solution = rrt.solve()
    feasible = solution.feasible
    print("RRT is feasible: ", feasible)
    #if feasible:
    #    for js in solution.x:
    #        config.setJointState(js)
    #        config.view(False, "RRT")
    #        time.sleep(0.01)

    return feasible

def propose_subgoals(x:Node, o:str, method:str="random", n:int=100, max_iter:int=10000): # Propose subgoals for the agent
    config       = ry.Config()
    config.addConfigurationCopy(x.C)
    max_x, max_y = config.frame("floor").getSize()[0:2]
    Z            = {}


    if method == "random":
        iter = 0 
        
        while len(Z) < n * 10 and iter < max_iter:                                      # Propose 10n subgoals
            config_base       = ry.Config()
            config_base.addConfigurationCopy(config)

            config_temp       = ry.Config()
            config_temp.addConfigurationCopy(config_base)
            

            px = np.random.uniform(-max_x/2, max_x/2)                # Generate random point
            py = np.random.uniform(-max_y/2, max_y/2)
            pn = [px, py, x.g[1][2]]
            
            col_base = config_temp.getCollisionsTotalPenetration()
        
            config_temp.frame(o).setPosition(pn)                      # Set the object position to the random point

            config_temp1       = ry.Config()
            config_temp1.addConfigurationCopy(config_temp)
            col_temp = config_temp1.getCollisionsTotalPenetration()
                

            if abs(col_base - col_temp) > 0.001:                               # Reject the point if it is in collision
                continue
            
            node = Node(config_base, [o, pn])

            Z[node] = score_function(node)
            iter += 1
    
    Z = sorted(Z, key=Z.get, reverse=True)[0:n]                # Sort the list using scoring function
    Z.append(Node(config, [o, [*config.frame(o).getPosition()[0:2], 0.1]])) # Add the original position as a subgoal

    #for i, z in enumerate(Z):
    #    config.addFrame(f"subgoal_p{i}", "world", "shape:ssBox, size:[0.2 0.2 .1 .005], color:[.3 1 .3 0.9], contact:0, logical:{table}").setPosition(z.g[1])                        # Add goal frame
    #config.view(True, "Subgoals")

    return Z                                                            # Return the top n subgoals
 
def rej(L: list, xf: ry.Config, O: list, threshold: float = 0.3):
    """
    Reject the node if it is similar to at most two nodes in L within a given threshold.

    Args:
        L (list): List of nodes to compare against.
        xf (ry.Config): Current configuration.
        O (list): List of object names.
        threshold (float): Maximum allowed distance to consider configurations similar.

    Returns:
        bool: True if the node is similar to at least two nodes in L, False otherwise.
    """
    def calc_pos(config: ry.Config, O: list):
        """Calculate positions of the movable objects and the agent."""
        obj_pos = []
        for o in O:
            pos = config.getFrame(o).getPosition()[0:2]
            obj_pos.append(pos)  # Keep positions as floating-point numbers
        agent_pos = config.getJointState()[0:2]
        return obj_pos, agent_pos

    def is_close(pos1, pos2, threshold):
        """Check if two positions are within the given threshold."""
        return np.linalg.norm(np.array(pos1) - np.array(pos2)) <= threshold

    similar_count = 0
    config = xf
    obj_pos, agent_pos = calc_pos(config, O)

    for l in L:
        config_temp = l.C
        obj_pos_temp, agent_pos_temp = calc_pos(config_temp, O)

        # Check if all object positions and agent positions are within the threshold
        obj_similarity = all(
            is_close(p1, p2, threshold) for p1, p2 in zip(obj_pos, obj_pos_temp)
        )
        agent_similarity = is_close(agent_pos, agent_pos_temp, threshold)

        if obj_similarity and agent_similarity:
            similar_count += 1
            if similar_count == 2:
                return True
            
    return False


def trace_back(x:Node, C0:ry.Config):                                             # Trace back the solution to the root node
    path = x.path
    
    for i, p in enumerate(path):

        if i != 0:
            if i % 2 == 0:
                C0.attach(x.agent, x.o)
            else:
                C0.frame(x.o).unLink()

        for pi in p:
            C0.setJointState(pi)
            C0.view(False, f"RRT {i}")
            time.sleep(0.005) 
         
        





        