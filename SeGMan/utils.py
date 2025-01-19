import robotic as ry
import time
import os
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

# TODO: Run komo for pick and then run RRT
def find_pick_path(C:ry.Config, agent:str, obj:str, FS: list, verbose: int):
    K = 20
    N = 20

    for k in range(K):
        # Find a feasible touch configuration
        if verbose > 0:
            print(f"Trying Pick KOMO for {k}")
        S = ry.Skeleton()
        S.enableAccumulatedCollisions(True)
        S.addEntry([0, 1], ry.SY.touch, [agent, obj])
        komo = S.getKomo_path(C, 1, 1e-1, 1e1, 1e-1, 1e2) 
        komo.initRandom()  
        ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
        if verbose > 2:
            komo.report(True, True, True)
            komo.view_play(True, f"Pick Komo Solution: {ret.feasible}")
            komo.view_close()
        if ret.feasible:
            for n in range(N):
                # Find feasible path between configurations
                if verbose > 0:
                    print(f"Trying Pick RRT for {n}")
                with suppress_stdout():
                    ry.params_clear()
                    ry.params_add({"rrt/stepsize": 0.1})
                    rrt = ry.PathFinder()                    
                    rrt.setProblem(C, C.getJointState(), komo.getPath()[-1])
                    s = rrt.solve()
                    ry.params_clear()
                    ry.params_add({"rrt/stepsize": 0.01})
                if s.feasible:
                    path = s.x
                    if verbose > 1:
                        C.view(True, "Pick Solution")
                    for p in path:
                        C.setJointState(p)
                        FS.append(C.getFrameState())
                        if verbose > 1:
                            C.view(False, "Pick Solution")
                            time.sleep(0.05)
                    C.view_close()
                    C.setJointState(path[-1])
                    return True
    return False

def find_place_path(C:ry.Config, goal:list, verbose: int):
    N = 20
    for n in range(N):
        # Find RRT path for object
        if verbose > 0:
            print(f"Trying Place RRT for {n}")

        with suppress_stdout():
            rrt = ry.PathFinder()                    
            rrt.setProblem(C, C.getJointState(), goal)
            s = rrt.solve()
        if s.feasible:
            path = s.x
            if verbose > 1:
                C.view(True, "Place Solution")
                for p in path:
                    C.setJointState(p)
                    C.view(False, "Place Solution")
                    time.sleep(0.05)
                C.view_close()
            return path, True
    return None, False

def solve_path(C:ry.Config, waypoint:list, agent:str, obj:str, FS: list, verbose: int):
    K = 20
    for k in range(K):
        C.addFrame("subgoal", "world", "shape: marker, size: [0.1]").setPosition([*waypoint, 0.2])
        if verbose > 0:
            print(f"Trying Move KOMO for {k}")
        S = ry.Skeleton()
        S.enableAccumulatedCollisions(True)
        S.addEntry([0.1, -1], ry.SY.touch, [agent, obj])
        S.addEntry([0.2, -1], ry.SY.stable, [agent, obj])
        S.addEntry([0.3, 0.4], ry.SY.positionEq, ["subgoal", obj])
        komo = S.getKomo_path(C, 10, 1e-5, 1e-3, 1e-5, 1e1)
        ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
        C.delFrame("subgoal")
        feasible = ret.eq < 1
        if verbose > 1 and not feasible:
            komo.report(True, True, True)
            komo.view_play(True, f"Move Komo Solution: {feasible}")
            komo.view_close()
        if feasible:
            C.setFrameState(komo.getPathFrames()[-1])
            FS.append(komo.getPathFrames())
            return True
    return False

def display_solution(C:ry.Config, FS:list):
    C.view(True, "Solution")
    for fs in FS:
        C.setFrameState(fs)
        C.view(False, "Solution")
        time.sleep(0.005)
        