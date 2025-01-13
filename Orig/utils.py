import robotic as ry
import numpy as np
from Node import Node
import time

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
    collisions = config.getCollisionsTotalPenetration()

    # Add a new object at the midpoint with the calculated orientation
    intermediate_obj = "intermediate_object"
    config.addFrame(intermediate_obj)\
        .setShape(ry.ST.ssBox,[size, 0.1,  .05, .005]).setPosition(np.array([midpoint[0], midpoint[1], 0.1])) \
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

    if abs(collisions1 - collisions) > 0.03:  
        return 0

    # If no collisions are detected, there is a line of sight
    return 1


def score_function(x:Node):
    
    goal   = x.g[1] 
    
    # Create a temporary configuration to add the intermediate object
    config = ry.Config()
    config.addConfigurationCopy(x.C)

    sg = config.addFrame("subgoal_mark", "world", "shape:ssBox, size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition(goal) 

    # The subgoal scoring heuristic
    v0 = is_line_of_sight(config, "goal", "subgoal_mark", False)   # Check line of sight between goal and object goal
    vg = is_line_of_sight(config, "subgoal_mark", x.agent, False)  # Check line of sight between object goal and agent
    
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
        
def solve(x:Node, g:list):                                          # Solve the task from the current configuration x to the end goal g
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
    S.addEntry([0.25, -1], ry.SY.stableOn, ["subgoal", obj])

    komo = S.getKomo_path(config, 100, 1e-3, 1e3, 1e-5, 1e3)
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()              # Solve

    #print(ret.eq, ret.ineq, ret.sos, ret.f)
    #r = komo.report(True, True, True)       
    #komo.view_play(True, str(ret.feasible), 0.3)

    if ret.feasible:
        #komo.view_play(True, str(ret.feasible), 0.3)
        komo_path = komo.getPath()
        q1 = komo_path[0]
        q2 = komo_path[10]
        q3 = komo_path[-2]

        config1 = ry.Config()
        config1.addConfigurationCopy(config)

        rrt1 = ry.PathFinder()                                           # Solve Bi-Directional RRT
        rrt1.setProblem(config1, [q1], [q2])
        solution1 = rrt1.solve()
        if solution1.feasible:

            config2 = ry.Config()
            config2.addConfigurationCopy(config1)
            
            config2.setJointState(solution1.x[-1])
            config2.attach(agent, obj)
            
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

def sample_points_around_object(C, agent, obj, num_points):
    center = C.frame(obj).getPosition()[:2]
    objSize = C.frame(obj).getSize()
    agentSize = C.frame(agent).getSize()
    inner_radius = objSize[0] / 2
    outer_radius = objSize[0] / 2 + agentSize[0] / 2 + 0.5

    # Randomly generate points in a cube first
    points = np.random.uniform(-1, 1, size=(num_points, 2))
    
    # Normalize the points to lie on a unit sphere
    points /= np.linalg.norm(points, axis=1)[:, np.newaxis]
    
    # Scale points to lie within the spherical shell (inner_radius to outer_radius)
    radii = np.random.uniform(inner_radius, outer_radius, size=num_points)
    points = center + points * radii[:, np.newaxis]
    
    return points
    

def sub_solve(x:Node, g:list):                                          # Solve the task from the current configuration x to the end goal g
    print("Running sub solve")
    config = ry.Config()
    config.addConfigurationCopy(x.C)
    agent  = x.agent
    obj    = g[0]
    goal   = g[1]
    p = x.path[:] 
    is_feas = False

    config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .05 .005], color:[1. .3 .3 0.9], contact:1, logical:{table}").setPosition(goal)                        # Add goal frame
    ry.params_add({'KOMO/verbose': 4})

    #qC =  C.getJointState()

    pairs = sample_points_around_object(config, agent, obj, 10)
    #print(pairs)

    for x, y in pairs:

        Ctemp = ry.Config()
        Ctemp.addConfigurationCopy(config)
    
        #x, y = sample_random_point(Ctemp) 
        Ctemp.setJointState([x,y])

        # go to object
        komo = ry.KOMO(Ctemp, 2, 1, 1, True)

        komo.addControlObjective([], 0, 1e-2)
        komo.addControlObjective([], 1, 1e-1)

        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.sos, scale=[1e1])


        komo.addModeSwitch([1.,2], ry.SY.stable, [agent, obj], True)
        komo.addObjective([1, 2], ry.FS.negDistance, [agent, obj], ry.OT.eq, [1e0])
        komo.addObjective([2], ry.FS.aboveBox, [obj, "subgoal"], ry.OT.ineq, [1e-1])
        
        #komo.initRandom() 

        solver = ry.NLP_Solver(komo.nlp(), verbose=0 )        # Solve
        ret = solver.solve() 
        path = komo.getPath()
        
        q_goal_pick = path[0]
        q_goal_place = path[-1]

        print("Pick feas: ", ret.feasible)
        if ret.feasible:
            rrt_pick = ry.PathFinder()                                           # Solve Bi-Directional RRT
            rrt_pick.setProblem(config, [config.getJointState()], [q_goal_pick])
            solution_pick = rrt_pick.solve()

            if solution_pick.feasible:
                #for js in solution_pick.x:
                #    config.setJointState(js)
                #    config.view(True, "Pick")
                #    time.sleep(0.01)

                config.setJointState(solution_pick.x[-1])
                config.attach(agent, obj)
                
                rrt_place = ry.PathFinder()                                           # Solve Bi-Directional RRT
                rrt_place.setProblem(config, [config.getJointState()], [q_goal_place])
                solution_place = rrt_place.solve()

                #config.view(True, "Place")

                
                if solution_place.feasible:
                    #for js in solution_place.x:
                    #    config.setJointState(js)
                    #    config.view(False, "Place")
                    #    time.sleep(0.01)
                    config.setJointState(solution_place.x[-1])
                    config.frame(obj).unLink()
                    p.append(solution_pick.x)
                    p.append(solution_place.x)
                    is_feas = True
                    break

    config.delFrame("subgoal")
    return Node(config, [obj, goal],path=p), is_feas

def reachable(x:Node, o:str):                                       # Return True if the agent can reach the object
    config       = ry.Config()
    config.addConfigurationCopy(x.C)

    q_agent  = config.getJointState()
    q_goal   = config.frame(o).getPosition()[0:2]
    obj = config.getFrame(o)
    del obj
    
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
            config_temp       = ry.Config()
            config_temp.addConfigurationCopy(config)
        
            config_base       = ry.Config()
            config_base.addConfigurationCopy(config)

            config_temp       = ry.Config()
            config_temp.addConfigurationCopy(config_base)
            

            px = np.random.uniform(-max_x/2, max_x/2)                # Generate random point
            py = np.random.uniform(-max_y/2, max_y/2)
            pn = [px, py, x.g[1][2]]
            
            col_base = config_base.getCollisions()
        
            config_temp.frame(o).setPosition(pn)                      # Set the object position to the random point

            config_temp1       = ry.Config()
            config_temp1.addConfigurationCopy(config_temp)
            col_temp = config_temp1.getCollisions(1e5)
                

            if len(col_base) != len(col_temp):                               # Reject the point if it is in collision
                continue
        
            node = Node(config_temp, [o, pn])

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
            
            
    print("Reject etmedim")

    print("Reject etmedim")
    return False


def trace_back(x:Node, C0:ry.Config):                                             # Trace back the solution to the root node
    path = x.path
    # print(len(path))
    for i, p in enumerate(path):

        if i != 0:
            if i % 2 == 0:
                C0.attach(x.agent, x.o)
            else:
                C0.frame(x.o).unLink()

        #if i != len(path):
        for pi in p:
            C0.setJointState(pi)
            C0.view(False, f"RRT {i}")
            time.sleep(0.005) 
        #else: 
        #    for pi in p:
        #        C0.setFrameState(pi)
        #        C0.view(False, f"RRT {i}")
        #        time.sleep(0.05) 
         