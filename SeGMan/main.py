import robotic as ry
from utils import (find_pick_path, find_place_path, solve_path, display_solution)


if __name__ == "__main__":
    C = ry.Config()
    C.addFile("config/p8-corner/p8-corner.g")
    C2 = ry.Config()
    C2.addFile("config/p8-corner/p8-corner-actuated.g")

    EGO_NAME = "ego"
    OBJ_NAME = "obj"
    GOAL     = C.getFrame("goal").getPosition()[0:2]
    C2.setJointState(C.getFrame(OBJ_NAME).getPosition()[0:2])
    C.view(True)
    C.view_close()

    FS = []
    found = False
    abort = False
    while not found and not abort:
        # Find a feasible pick configuration and path
        f = find_pick_path(C, EGO_NAME, OBJ_NAME, FS, 1)
 
        # If it is not possible to go check if there is an obstacle that can be removed
        if not f: 
            # Run SeGMan
            # TODO: Implement SeGMan
            Ns, fs = None, False
            if not fs:
                break
            else:
                continue
        
        # Check for object path
        C2.setJointState(C.frame(OBJ_NAME).getPosition()[0:2])
        P, fr = find_place_path(C2, GOAL, 1)

        # If it is not possible to go check if there is an obstacle that can be removed
        if not fr: 
            # Run SeGMan
            # TODO: Implement SeGMan
            Ns, fs = None, False
            if not fs:
                abort = True
                break
            else:
                continue
        
        # Follow the RRT path with KOMO
        for pi, wp in enumerate(P):
            print(f"{pi} / {len(P)-1}")
            fk = solve_path(C, wp, EGO_NAME, OBJ_NAME, FS, 1)
            if not fk:
                break
            if pi == len(P)-1:
                found = True

    if found:
        print("Solution found!")
        display_solution(C, FS)
    else:
        print("Solution not found!")
    




