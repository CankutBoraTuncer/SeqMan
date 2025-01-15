import robotic as ry
from utils import (select_node, solve, sub_solve, reachable, propose_subgoals, rej, trace_back)
from Node import Node

if __name__ == "__main__":
    task = "SeqMan-main/manipulation-puzzles-main/puzzles/p3-maze.g"
    EGO_NAME = "ego"
    OBJ_NAME = "obj"

    C0 = ry.Config()
    C0.addFile(task)
    C0.view(True)
    C0.view_close()
    
    ry.params_add({
        "rrt/stepsize": 0.01,
        "rrt/verbose": -1,
    })

    O = [OBJ_NAME] #"obj1", "obj2", "obj3", "obj4"]                  # Objects
    G = [OBJ_NAME, [*C0.frame("goal_visible").getPosition()[0:2], 0.1]]  # Main Goal
    Node.main_goal = G              # Set the main goal
    L = [Node(C0, G, path=[[]], agent_name=EGO_NAME)]     # List of nodes
    
    is_solved = False               # Solution found flag
    try_count = 0                   # Number of tries

    while len(L) > 0:
        try_count += 1
        print(f"-----------------------------------------------Try count {try_count}-----------------------------------------------")
        x = select_node(L)                                              # Select the node using feasiblitiy heuristic
        print(f"Remaining node count: {len(L)}")
        print(f"Selected node: {x}")

        if x == None:                                                   # Abort if no node is selected
            break

        print("Testing end goal")
        X, feasible = sub_solve(x, False)                                       # Try to reach the goal

        if feasible:
            X.C.view(True, "Solution found")
            trace_back(X, C0)              # Trace the solution back to x0
            print("Solution found")
            is_solved = True
            break
            
        for o in O:      
            if not reachable(x, o):                                  # Check if agent can reach the object
                continue
            
            print("Generating subgoals")
            Z = propose_subgoals(x, o, method="random", n=100)          # Propose subgoals

            for i, z in enumerate(Z):
                print(f"Subgoal {i+1}/{len(Z)} | try count {try_count} | Node: {z}", end="")  
                xf, feasible = sub_solve(z, False) 
                print(f" | Feasible: {feasible}")  
                 
                
                if feasible and not rej(L, xf.C, O):                      # Check if subgoal is config is feasible and not rejected
                    L.append(xf)                                           # Add the subgoal to the list of nodes

    if not is_solved:
        print("No solution found")
