import robotic as ry
from utils import (select_node, solve, sub_solve, reachable, propose_subgoals, rej)
from Node import Node

if __name__ == "__main__":
    task = "../config/p9-cube-free.g"
    EGO_NAME = "ego"
    OBJ_NAME = "obj"

    C0 = ry.Config()
    C0.addFile(task)
    print(C0.frame("ego").getPosition(), C0.frame("obj").getPosition())
    C0.view(True)

    ry.params_add({
        "rrt/stepsize": 0.01,
        "rrt/verbose": -1
    })

    O = [OBJ_NAME]                  # Objects
    G = [OBJ_NAME, C0.frame("goal_visible").getPosition()[0:2]]  # Main Goal
    Node.main_goal = G              # Set the main goal
    L = [Node(C0, G, EGO_NAME)]     # List of nodes
    
    is_solved = False               # Solution found flag

    while len(L) > 0:
        
        x = select_node(L)                                              # Select the node using feasiblitiy heuristic
                                                     
        if x == None:                                                   # Abort if no node is selected
            break

        X, feasible = solve(x, G)                                       # Try to reach the goal

        if feasible:
            # TODO: Implement the path generation function              # Trace the solution back to x0
            print("Solution found")
            X.view(True)
            is_solved = True
            break
            
        for o in O:
            print("If reachable")
            if not reachable(x, o):                                     # Check if agent can reach the object
                continue
            
            Z = propose_subgoals(x, o, method="random", n=3)          # Propose subgoals

            for z in Z:
                xf, feasible = sub_solve(x, z.g)                               # Solve the subgoal
                
                if feasible and not rej(L, xf, O):                      # Check if subgoal is config is feasible and not rejected
                    L.append(Node(xf, [o, z.g]))
        is_first = False
    if not is_solved:
        print("No solution found")





