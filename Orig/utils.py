import robotic as ry
import numpy as np
from Node import Node
import time

def is_line_of_sight(C, obj, goal):
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
        .setShape(ry.ST.ssBox,[size, 0.1,  .05, .005]).setPosition(np.array([midpoint[0], midpoint[1], obj_pos[2]])) \
        .setContact(1)\
        .setQuaternion([np.cos(angle / 2), 0, 0, np.sin(angle / 2)])  # Rotation around Z-axis
    #config.view(True)

    # Check for collisions
    config1 = ry.Config()
    config1.addConfigurationCopy(config)
    collisions1 = config1.getCollisionsTotalPenetration()
    
    del config
    del config1

    if abs(collisions1 - collisions) > 0.05:  
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
    v0 = is_line_of_sight(config, "goal", "subgoal_mark")   # Check line of sight between goal and object goal
    vg = is_line_of_sight(config, "subgoal_mark", x.agent)  # Check line of sight between object goal and agent
    
    config.delFrame("subgoal_mark")

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
    sg = config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .05 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition(goal)                        # Add goal frame
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


    

def sub_solve(x:Node, g:list):                                          # Solve the task from the current configuration x to the end goal g
    print("Running sub solve")
    config = ry.Config()
    config.addConfigurationCopy(x.C)
    agent  = x.agent
    obj    = g[0]
    goal   = g[1]
    p = x.path[:] 
    is_feas = False

    config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .05 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition(goal)                        # Add goal frame

    komo_pick = ry.KOMO(config, phases=1, slicesPerPhase=1, kOrder=0, enableCollisions=True)                            # Initialize LGP
    komo_pick.initRandom(2)                                                                                              # Randomize the initial configuration
    komo_pick.addObjective([], ry.FS.negDistance, [agent, obj], ry.OT.eq, scale=1e2, target=0)                                     # Pick
    komo_pick.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e3) 
    ret_pick = ry.NLP_Solver(komo_pick.nlp(), verbose=0).solve()              # Solve

    q_goal_pick = komo_pick.getPath_qAll()[-1]

    print("Pick feas: ", ret_pick.feasible)

    rrt_pick = ry.PathFinder()                                           # Solve Bi-Directional RRT
    rrt_pick.setProblem(config, [config.getJointState()], [q_goal_pick])
    solution_pick = rrt_pick.solve()

    if solution_pick.feasible and ret_pick.feasible:
        #for js in solution_pick.x:
        #    config.setJointState(js)
        #    config.view(False, "Pick")
        #    time.sleep(0.01)

        config.setJointState(solution_pick.x[-1])
        config.attach(agent, obj)

        komo_place = ry.KOMO(config, phases=1, slicesPerPhase=5, kOrder=0, enableCollisions=True)                            # Initialize LGP
        komo_place.addObjective([] , ry.FS.aboveBox, [obj, "subgoal"], ry.OT.ineq, scale=1e2)  # Place constraints         
        komo_place.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e3) 
        ret_place = ry.NLP_Solver(komo_place.nlp(), verbose=0).solve()

        q_goal_place = komo_place.getPath_qAll()[-1]

        print("Place feas: ", ret_place.feasible)
        
        rrt_place = ry.PathFinder()                                           # Solve Bi-Directional RRT
        rrt_place.setProblem(config, [config.getJointState()], [q_goal_place])
        solution_place = rrt_place.solve()

        #config.view(True, "Place")

        
        if solution_place.feasible and ret_place.feasible:
            #for js in solution_place.x:
            #    config.setJointState(js)
            #    config.view(False, "Place")
            #    time.sleep(0.01)
            config.setJointState(solution_place.x[-1])
            config.frame(obj).unLink()
            p.append(solution_pick.x)
            p.append(solution_place.x)
            is_feas = True
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
            config_base       = ry.Config()
            config_base.addConfigurationCopy(config)

            config_temp       = ry.Config()
            config_temp.addConfigurationCopy(config)

            px = np.random.uniform(-max_x/2, max_x/2)                # Generate random point
            py = np.random.uniform(-max_y/2, max_y/2)
            pn = [px, py, 0.2]

            col_base = config_base.getCollisions()
        
            config_temp.frame(o).setPosition(pn)                      # Set the object position to the random point

            col_temp = config_temp.getCollisions(1e5)
                

            if len(col_base) != len(col_temp):                               # Reject the point if it is in collision
                continue
        
            node = Node(config_base, [o, pn])

            Z[node] = score_function(node)
            iter += 1
    
    Z = sorted(Z, key=Z.get, reverse=True)[0:n]                # Sort the list using scoring function
    
    #for i, z in enumerate(Z):
    #    config.addFrame(f"subgoal_p{i}", "world", "shape:ssBox, size:[0.2 0.2 .1 .005], color:[.3 1 .3 0.9], contact:0, logical:{table}").setPosition([*z.g[1], 0.1])                        # Add goal frame
    #config.view(True, "Subgoals")

    #Z.append(Node(config, [o, op]))                             # Add the original position as a subgoal
    return Z                                                            # Return the top n subgoals
 
def rej(L:list, xf:ry.Config, O:list):                                    # Reject the node if it is similar to the at most two nodes in L
    def calc_pos(config:ry.Config, O:list):                         # Calculate discretized position of the movable objects and agent
        obj_pos = []
        for o in O:
            pos = config.getFrame(o).getPosition()[0:2]
            obj_pos.append([round(pos[0], 2), round(pos[1], 2)])
        agent_pos = config.getJointState()[0:2]
        agent_pos = [round(agent_pos[0], 2), round(agent_pos[1], 2)]
        return obj_pos, agent_pos
    
    similar_count       = 0
    config              = xf
    obj_pos, agent_pos  = calc_pos(config, O)

    for l in L:
        config_temp = l.C
        obj_pos_temp, agent_pos_temp = calc_pos(config_temp, O)

        if np.all(obj_pos == obj_pos_temp) and np.all(agent_pos == agent_pos_temp):
            similar_count += 1
            if similar_count == 2:
                return True

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
         
        





        
    