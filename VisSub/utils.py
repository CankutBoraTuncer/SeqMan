import robotic as ry
import numpy as np
import matplotlib.pyplot as plt
from Node import Node
import time
import os
from contextlib import contextmanager
import base64
from openai import OpenAI

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

    # Create a temporary configuration to add the intermediate object
    config = ry.Config()
    config.addConfigurationCopy(C)
    collisions = config.getCollisionsTotalPenetration()

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

    config.addFrame("subgoal_mark", "world", "shape:ssBox, size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition([*goal[0:2], 0.1])                        # Add goal frame

    # The subgoal scoring heuristic
    vg = is_line_of_sight(config, "subgoal_mark", x.agent, False)  # Check line of sight between object goal and agent

    config.delFrame(x.agent)
    # the agent is not counted as an obstacle
    v0 = is_line_of_sight(config, "goal", "subgoal_mark", False)   # Check line of sight between goal and object goal
    
    
    final_goal_pos = config.getFrame("goal").getPosition()

    # proximity d of object goal to the goal, and is set to 5 if d < 0.2, 2 if d < 0.4, and 0 otherwise.
    d = np.linalg.norm(goal[:2] - final_goal_pos[:2])

    vdist = 0

    if d < 2:
        vdist = 5
    elif d < 4:
        vdist = 2
    x.score = 10*v0 + 5*vg + vdist    


def select_node(L:list):                                            # Returns the node with the lowest cost
    min_score = np.inf
    min_node  = None
    t         = 0                                                   # Min try count 

    while t <= 2:                                                    # Each node can be tried twice 
        for x in L:        
            if x.t == t:
                if x.score < 0:
                    score_function(x)
                if x.score  < min_score:                              # Node with the least try count has more priority
                    min_score = x.score
                    min_node = x

        if min_node is not None:                                    # Leave if a node is found
            break
        t += 1

    if min_node is None:
        return None
    
    min_node.t += 1                                                 # Increment the number of times this node is tried
    return min_node
        
def solve(x:Node, view:bool=False):                                          # Solve the task from the current configuration x to the end goal g
    config = ry.Config()
    config.addConfigurationCopy(x.C)
    agent  = x.agent
    obj    = x.o
    goal   = x.og
    config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .05 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition([*goal[0:2], 0.0])                        # Add goal frame
    p = x.path[:]
    ln = x.layer_no

    S = ry.Skeleton()
    S.enableAccumulatedCollisions(True)

    S.addEntry([0, -1], ry.SY.touch, [agent, obj])
    S.addEntry([0.5, -1], ry.SY.stable, [agent, obj])
    S.addEntry([0.8, -1], ry.SY.above, [obj, "subgoal"])

    komo = S.getKomo_path(config, 30, 1e-1, 1e1, 1e-1, 1e2)
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()              # Solve

    #print(ret.eq, ret.ineq, ret.sos, ret.f)
    #r = 
    if view:   
        komo.view_play(True, 0.3)

    if ret.feasible:
        
        #komo.view_play(True, str(ret.feasible), 0.3
        config.delFrame("subgoal")
        komo_path = komo.getPathFrames()
        p.append(komo_path)
        config.setFrameState(komo_path[-1])

    return Node(config, Node.main_goal, path=p, layer_no=ln, score=x.score), ret.feasible

def sub_solve(x:Node, view:bool=False):                                          # Solve the task from the current configuration x to the end goal g
    config = ry.Config()
    config.addConfigurationCopy(x.C)
    agent  = x.agent
    obj    = x.o
    goal   = x.og
    config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .05 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition([*goal[0:2], 0.0])                        # Add goal frame
    p = x.path[:]
    ln = x.layer_no
    komo_path = []

    komo = ry.KOMO(config, phases=2, slicesPerPhase=1, kOrder=1, enableCollisions=True)                            # Initialize LGP
    komo.initRandom()       
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e3)                                                                                        # Randomize the initial configuration
    komo.addObjective([0,1], ry.FS.distance, [agent, obj], ry.OT.eq, scale=1e2, target=0)                                     # Pick
    komo.addModeSwitch([1,2], ry.SY.stable, [agent, obj], True)   
    komo.addObjective([1,2] , ry.FS.aboveBox, [obj, "subgoal"], ry.OT.ineq, scale=1e1)  # Place constraints 

    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()              # Solve

    #print(ret.eq, ret.ineq, ret.sos, ret.f)
    #r = komo.report(True, True, True)   
    if view :#and ret.feasible:    
        komo.view_play(True, str(ret.feasible), 0.3)
        komo.view_close()

    path = komo.getPath()

    q0 = config.getJointState()
    qT = path[0]

    
    config1, solution1 = solveRRT(config, q0, qT)

    if solution1.feasible:
        path1 = getFrames(config1, solution1, view)
        komo_path += path1
        
        config1.attach(agent, obj)

        q0 = config1.getJointState()
        qT = path[1]
        
        config2, solution2 = solveRRT(config1, q0, qT)

        if solution2.feasible:
            path2 = getFrames(config2, solution2, view)
            komo_path += path2
            config2.frame(obj).unLink()
            config2.delFrame("subgoal")
            p.append(komo_path)

        return Node(config2, Node.main_goal, path=p, layer_no=ln, score=x.score), True
    return None, False

def getFrames(config, solution, view):
    komo_path = []
    if view:
        config.view(True, "Pick RRT")
    for js in solution.x:
        config.setJointState(js)
        if view:
            config.view(False)
            time.sleep(0.005)
        komo_path.append(config.getFrameState())
    config.view_close()
    return komo_path

def solveRRT(C, q0, qT):
    config = ry.Config()
    config.addConfigurationCopy(C)
    with suppress_stdout():
        rrt = ry.PathFinder()                                           # Solve Bi-Directional RRT
        rrt.setProblem(config, [q0], [qT])
        solution = rrt.solve()
    return config, solution

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

    return solution.feasible

def propose_subgoals(x:Node, o:str, method:str="random", n:int=100, max_iter:int=5000): # Propose subgoals for the agent
    config       = ry.Config()
    config.addConfigurationCopy(x.C)
    max_x, max_y = config.frame("floor").getSize()[0:2]
    Z            = {}


    if method == "random":
        iter = 0 
        
        while len(Z) < n * 10 and iter < max_iter:                                      # Propose 10n subgoals       
            config_base       = ry.Config()
            config_base.addConfigurationCopy(config)            

            px = np.random.uniform(-max_x/2, max_x/2)                # Generate random point
            py = np.random.uniform(-max_y/2, max_y/2)
            pn = [px, py, x.g[1][2]]
            
            config_temp       = ry.Config()
            config_temp.addConfigurationCopy(config_base)

            col_pre = config_temp.getCollisions()

            config_temp.frame(o).setPosition(pn)                      # Set the object position to the random point

            config_temp2       = ry.Config()
            config_temp2.addConfigurationCopy(config_temp)

            col_post = config_temp2.getCollisions(1e5)
                
            if len(col_pre) != len(col_post):                               # Reject the point if it is in collision
                continue
        
            node = Node(config_base, [o, pn], path=x.path, layer_no=(x.layer_no+1))  # Create a new node
            score_function(node)
            Z[node] = node.score
            iter += 1
    
    Z = sorted(Z, key=Z.get, reverse=True)[0:n]                # Sort the list using scoring function
    Z.append(Node(x.C, x.g, path=x.path, layer_no=(x.layer_no+1), score = x.score)) # Add the original position as a subgoal

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

        for pi in p:
            C0.setFrameState(pi)
            C0.view(False, f"View {i}")
            time.sleep(0.02) 


# Function to get the OpenAI client
def get_client(api_key: str) -> OpenAI:
    return OpenAI(api_key=api_key)


# Function to encode the image
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")


def overlay_grid_over_image(C0:ry.Config, grid_size:int, camera_name:str="camera-top", camera_path:str="config/camera-top.g"):
    config = ry.Config()
    config.addConfigurationCopy(C0)
    print(camera_path)
    config.addFile(camera_path)
    config.view()
    config.view_setCamera(config.getFrame(camera_name))
    config.view(True)
    config.view_savePng("tmp/tmp")
    config.view_close()
    img = plt.imread("tmp/tmp0000.png")
    os.remove("tmp/tmp0000.png")

    fig = plt.figure(figsize=(img.shape[1] / 100, img.shape[0] / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    
    ax.set_xticks(np.arange((img.shape[1]%grid_size)/2, img.shape[1], grid_size))
    ax.set_yticks(np.arange((img.shape[0]%grid_size)/2, img.shape[0], grid_size))
    ax.grid(color='black', linestyle='-', linewidth=0.5)
    
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    
    fig.canvas.draw()
    modified_image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    modified_image = modified_image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    
    plt.close(fig)
    
    return modified_image

