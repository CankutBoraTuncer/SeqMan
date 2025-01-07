import robotic as ry
import numpy as np
from Node import Node
import time

def score_function(x:Node):                                         # The subgoal scoring heuristic
    v0 = 1                                                          # Check line of sight between goal and object goal
    vg = 1                                                          # Check line of sight between object goal and agent
    vdist = 1                                                       # proximity d of object goal to the goal, and is set to 5 if d < 0.2, 2 if d < 0.4, and 0 otherwise.
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
    goal   = [*g[1], 0.2]
    sg = config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition(goal)                        # Add goal frame

    S = ry.Skeleton()
    S.enableAccumulatedCollisions(True)

    S.addEntry([0, 0.1], ry.SY.touch, [agent, obj])
    S.addEntry([0.1, 0.25], ry.SY.stable, [agent, obj]) 
    S.addEntry([0.2, -1], ry.SY.above, [obj, "subgoal"])
    S.addEntry([0.25, -1], ry.SY.stableOn, ["floor", obj])

    komo = S.getKomo_path(config, 30, 1e-3, 1e3, 1e-5, 1e3)
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()              # Solve

    #print(ret.eq, ret.ineq, ret.sos, ret.f)
    r = komo.report(True, True, True)       
    komo.view_play(True, str(ret.feasible), 0.3)

    pf = komo.getPathFrames()[-1]
    config.setFrameState(pf)
    del sg

    if ret.feasible:
        config.view(True, "Feas Config")

    return config, ret.feasible

def sub_solve(x:Node, g:list):                                          # Solve the task from the current configuration x to the end goal g
    print("Running sub solve")
    config = ry.Config()
    config.addConfigurationCopy(x.C)
    agent  = x.agent
    obj    = g[0]
    goal   = [*g[1], 0.2]
    config.addFrame("subgoal", "world", "shape:ssBox, size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table}").setPosition(goal)                        # Add goal frame

    komo1 = ry.KOMO(config, phases=1, slicesPerPhase=5, kOrder=0, enableCollisions=True)                            # Initialize LGP
    komo1.addObjective([], ry.FS.distance, [obj, agent], ry.OT.eq, scale=1e2)                                     # Pick
    komo1.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2) 
    ry.NLP_Solver(komo1.nlp(), verbose=0).solve()              # Solve
    config.setFrameState(komo1.getPathFrames()[-1])
    q_agent = config.getJointState()
    config.attach(agent, obj)
    config.view(True, "Pick")

    komo2 = ry.KOMO(config, phases=1, slicesPerPhase=5, kOrder=0, enableCollisions=True)                            # Initialize LGP
    komo2.addObjective( [0,-1] , ry.FS.aboveBox, [obj, "subgoal"], ry.OT.ineq, scale=1e3)  # Place constraints         
    komo2.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2) 
    ry.NLP_Solver(komo2.nlp(), verbose=0).solve() 
    config.setFrameState(komo2.getPathFrames()[-1])

    q_goal = komo2.getPath_qAll()[-1][0:2]
    config.view(True, "Place")

    print(q_agent, q_goal)
    rrt = ry.PathFinder()                                           # Solve Bi-Directional RRT
    rrt.setProblem(config, [q_agent], [q_goal], collisionTolerance=0.01)
    solution = rrt.solve()
    feasible = solution.feasible

    if feasible:
        for js in solution.x:
            config.setJointState(js)
            config.view(False, "RRT")
            time.sleep(0.01)

    config.setJointState(solution.x[-1])
    config.frame(obj).unLink()

    return config, feasible

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
    pn = x.og[0:2]
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
    if feasible:
        for js in solution.x:
            config.setJointState(js)
            config.view(False, "RRT")
            time.sleep(0.01)

    return feasible

def propose_subgoals(x:Node, o:str, method:str="random", n:int=100): # Propose subgoals for the agent
    config       = ry.Config()
    config.addConfigurationCopy(x.C)
    agent        = x.agent
    max_x, max_y = config.frame("floor").getSize()[0:2]
    Z            = {}
    obj          = config.getFrame(o)    
    op           = obj.getPosition()

    if method == "random":

        while len(Z) < n * 10:                                      # Propose 10n subgoals
            config_base       = ry.Config()
            config_base.addConfigurationCopy(config)

            config_temp       = ry.Config()
            config_temp.addConfigurationCopy(config)

            config_temp.computeCollisions()
            col_p = config_temp.getCollisions()

            px = np.random.uniform(-max_x/2, max_x/2)                # Generate random point
            py = np.random.uniform(-max_y/2, max_y/2)
            pn = [px, py, 0.2]
        
            obj.setPosition(pn) 
                                                    # Set the object position to the random point
            config_temp.computeCollisions()
            col = config_temp.getCollisions()

            if len(col) != len(col_p):                               # Reject the point if it is in collision
                continue

            Z[Node(config_base, [o, [px, py]])] = score_function(Node(config_temp, [o, []]))

    
    Z = sorted(Z, key=Z.get, reverse=True)[0:n]                # Sort the list using scoring function
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
        print(l)
        config_temp = l.C
        obj_pos_temp, agent_pos_temp = calc_pos(config_temp, O)

        if np.all(obj_pos == obj_pos_temp) and np.all(agent_pos == agent_pos_temp):
            similar_count += 1
            if similar_count == 2:
                return True

    return False