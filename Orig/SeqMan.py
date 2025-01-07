import robotic as ry
from utils import (select_node, solve, reachable, propose_subgoals, rej)
from Node import Node

if __name__ == "__main__":
    task = "../config/p8-corner.g"
    EGO_NAME = "ego"
    OBJ_NAME = "obj"

    C0 = ry.Config()
    C0.addFile(task)
    C0.view(True)

    ry.params_add({
        "rrt/stepsize": 0.01,
        "rrt/verbose": -1
    })

    O = [OBJ_NAME]                  # Objects
    G = [OBJ_NAME, "goal_visible"]  # Main Goal
    Node.main_goal = G              # Set the main goal
    L = [Node(C0, G, EGO_NAME)]     # List of nodes
    
    is_solved = False               # Solution found flag

    while len(L) > 0:
        
        x = select_node(L)                                              # Select the node using feasiblitiy heuristic

        X, feasible = solve(x, G)                                       # Try to reach the goal

        if feasible:
            # TODO: Implement the path generation function              # Trace the solution back to x0
            is_solved = True
            break
            
        for o in O:
            if not reachable(x, o):                                     # Check if agent can reach the object
                continue
            
            Z = propose_subgoals(x, o, method="random", n=100)          # Propose subgoals

            for z in Z:
                g = [o, z]
                X, feasible = solve(x, g)                               # Solve the subgoal
                xf = X[-1]
                
                if feasible and not rej(L, xf, O):                      # Check if subgoal is config is feasible and not rejected
                    L.append(xf)

    if not is_solved:
        print("No solution found")




