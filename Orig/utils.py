import robotic as ry
import numpy as np
from Node import Node

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
    
    min_node.c += 1                                                 # Increment the number of times this node is tried
    return min_node
        
def solve(x:Node, g:list):                                          # Solve the task from the current configuration x to the end goal g
    config = x.C
    agent  = x.agent
    obj    = g[0]
    goal   = g[1]

    
    komo = ry.KOMO(config, phases=1, slicesPerPhase=1, kOrder=1, enableCollisions=True) # Initialize LGP
                                                                    # Pick constaints
    komo.addModeSwitch([], ry.SY.touch,  [agent, obj])              
    komo.addModeSwitch([], ry.SY.stable, [agent, obj])
                                                                    # Place constraints
    komo.addModeSwitch([], ry.SY.above,  [agent, goal])             
    komo.addModeSwitch([], ry.SY.stable, [agent, "floor"])

   
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)  # Collision avoidance

    
    ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()              # Solve

    return komo.getPathFrames(), ret.feasible

def reachable(x:Node, o:str):                                       # Return True if the agent can reach the object
    config   = x.C
    q_agent  = config.getJointState()
    q_goal   = config.frame(o).getPosition()[0:1]

    
    rrt = ry.PathFinder()                                           # Solve Bi-Directional RRT
    rrt.setProblem(config, [q_agent], [q_goal], collisionTolerance=0.01)
    solution = rrt.solve()

    return solution.feasible


def propose_subgoals(x:Node, o:str, method:str="random", n:int=100): # Propose subgoals for the agent
    config       = ry.Config()
    config.addConfigurationCopy(x.C)
    agent        = x.agent
    max_x, max_y = config.frame("floor").getSize()[0:1]
    Z = []

    if method == "random":

        while len(Z) < n * 10:                                      # Propose 10n subgoals

            
            config.computeCollisions()                              # Check configuration before moving
            collisions_old = config.getCollisions()

            
            x = np.random.uniform(-max_x/2, max_x/2)                # Generate random point
            y = np.random.uniform(-max_y/2, max_y/2)
            config.frame(agent).setPosition([0, 0, 0.2])

            
            config.computeCollisions()                              # Check if random point is feasible
            collisions_new = config.getCollisions()
            
            if len(collisions_new) <= len(collisions_old):
                Z.append(Node(x.C, [o, [x, y]], agent))
    

    Z = sorted(Z, key=score_function, reverse=True)                 # Sort the list using scoring function

    return Z[0:n]                                                   # Return the top n subgoals
 
def rej(L:list, xf:ry.Config, O:list):                                    # Reject the node if it is similar to the at most two nodes in L
    def calc_pos(config:ry.Config, O:list):                         # Calculate discretized position of the movable objects and agent
        obj_pos = []
        for o in O:
            pos = config.getFrame(o).getPosition()[0:1]
            obj_pos.append([round(pos[0], 2), round(pos[1], 2)])
        agent_pos = config.getJointState()[0:1]
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