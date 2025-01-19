import robotic as ry
import time
import os
from contextlib import contextmanager
from itertools import combinations
from src.Node import Node
import numpy as np
import matplotlib.pyplot as plt
import math

class SeGMan():
    def __init__(self, C:ry.Config, C_hm:ry.Config, agent:str, obj:str, goal:list, obs_list:list, verbose:int):
        self.C = C
        self.C_hm = C_hm
        self.agent = agent
        self.obj = obj
        self.goal = goal
        self.obs_list = obs_list
        self.verbose = verbose
        self.FS = []
        self.OP = []

    def run(self):
        found = False
        C2 = self.make_agent(self.C, self.obj)
        while not found:
            # Find a feasible pick configuration and path
            f = self.find_pick_path(self.C, self.agent, self.obj, self.FS, self.verbose, 1, 1)
    
            # If it is not possible to go check if there is an obstacle that can be removed
            if not f: 
                # Remove obstacle
                fs = self.remove_obstacle(0)
                if not fs:
                    return
            
            # Check for object path
            C2.setJointState(self.C.frame(self.obj).getPosition()[0:2])
            P, fr = self.find_place_path(C2, self.goal, 1)

            # If it is not possible to go check if there is an obstacle that can be removed
            if not fr: 
                # Remove obstacle
                fs = self.remove_obstacle(1)
                if not fs:
                    return
            
            # Follow the RRT path with KOMO
            found = self.solve_path(self.C, P, self.agent, self.obj, self.FS, 1)

        if found:
            print("Solution found!")
            return True
        else:
            print("Solution not found!")
            return False

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def remove_obstacle(self, type:int):
        # Type 0: Agent cannot reach obj, Type: 1 Obj cannot reach goal

        # Generate obstacle pair
        self.generate_obs_pair()

        # Check which pairs are the source of the collision
        self.find_collision_pair(type)

        max_iter = 500
        idx = 0
        N = []
        for op in self.OP:
            N.append(Node(self.C, op, layer=0))

        while len(N) > 0 and idx > max_iter:
            idx+=1

            # Select the best node
            node = self.select_node(N)

            # Check if configuration is feasible
            f = False
            if type==0:
                f = self.find_pick_path(node.C, self.agent, self.obj, node.FS, 0, 2, 2)
                if f:
                    return node.FS, True
            else:
                P, f = self.find_place_path(node.C, node.C.frame(self.obj).getPosition()[0:2], 0, 2)
                if f:
                    return P, True    
            
            # Check which objects are reachable in the pair
            any_reach = False
            for o in node.op:
                if not self.is_reachable(node.C, o):
                    continue
                any_reach = True

                # For the reachable object, generate subgoals
                # TODO: Complete generate_subgoal
                Z = self.generate_subgoal()

                # For each subgoal try to pick and place
                for z in Z:
                    C2 = self.make_agent(node.C, o)
                    P, f1 = self.find_place_path(C2, z, 0, 2)
                    if f1: 
                        feas = self.solve_path(self.C, P, self.agent, self.obj, self.FS, 1)
                        if feas:
                            # Calculate the scene score
                            node_score = self.node_score(node)
                            N.append(Node(z.c, node.op, node, [], node.layer+1, score= node_score))

            if not any_reach:
                N.remove(node)

        return None, False

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def node_score(self, node:Node):
        Ct = ry.Config()
        Ct.addConfigurationCopy(self.C3)

        frame_names = node.C.getFrameNames()
        for fn in frame_names:
            if "obj" in fn:
                Ct.frame(fn).setPosition(node.C.frame(fn).getPosition())
        
        if self.verbose > 1:
            Ct.view(True, "Changed heatmap")
        
        scene_score = self.scene_score(Ct)
        layer_score = node.layer
        obj_score   = len(node.op)

        return scene_score / (math.sqrt(layer_score * obj_score))

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def make_agent(self, C:ry.Config, obj:str):
        C2 = ry.Config()
        C2.addConfigurationCopy(C)
        frame_names = C2.getFrameNames()
        for fn in frame_names:
            if "ego" in fn:
                C2.delFrame(fn)

        C2.frame(obj).setJoint(ry.JT.transXY, limits=[-4, 4, -4, 4])
        return C2
   
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def generate_subgoal(self):
        return None

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def is_reachable(self, C:ry.Config, o:str):
        return self.find_pick_path(C, self.agent, o, [], 0, 2, 2)
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def select_node(N:list):
        best_score = float('-inf')
        best_node = None
        for node in N:
            if node.score > best_score:
                best_node = node
                best_score = node.score
        return best_node
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def generate_obs_pair(self):
        for r in range(1, len(self.obs_list) + 1):
            self.OP.extend(combinations(self.obs_list, r))
        self.OP = [list(item) for item in self.OP]

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
    def find_collision_pair(self, type:int):
        for op in self.OP:
            Ct = ry.Config()
            Ct.addConfigurationCopy(self.C)
            if self.verbose > 0:
                print(f"Trying: {op}")
            for o in op:
                Ct.frame(o).setContact(0)
            f = False
            if type == 0:
                f = self.find_pick_path(Ct, self.agent, self.obj, [], 0, 2, 2)
            else:
                f = self.find_place_path(Ct, Ct.frame(self.obj).getPosition()[0:2], 0, 2)
            if self.verbose > 0:
                print(f"Is {op} blocking path: {f}")
            if not f:
                self.OP.remove(op)
        print(f"The blocking obstacle pairs: {self.OP}")

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_pick_path(self, C:ry.Config, agent:str, obj:str, FS:list, verbose: int, K:int=20, N:int=20):
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
                    with self.suppress_stdout():
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

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_place_path(self, C:ry.Config, goal:list, obj:str, verbose: int, N:int = 20):
        for n in range(N):
            # Find RRT path for object
            if verbose > 0:
                print(f"Trying Place RRT for {n}")

            with self.suppress_stdout():
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

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def solve_path(self, C:ry.Config, P:list, agent:str, obj:str, FS:list, verbose: int):
        for pi, wp in enumerate(P):
            if self.verbose > 0:
                print(f"{pi} / {len(P)-1}")
            K = 20
            for k in range(K):
                C.addFrame("subgoal", "world", "shape: marker, size: [0.1]").setPosition([*wp, 0.2])
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
                    break
                if k == K-1:
                    return False
        return True

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def scene_score(self, C:ry.Config, cam_frame:str):
        img = SeGMan.get_image(C, cam_frame, self.verbose)
        scene_score = 0
        for r in img:
            for rgb in r:
                if rgb[0] > 200 and rgb[1] < 200 and rgb[2] < 200:
                    scene_score += 1
                elif rgb[1] > 200 and rgb[0] > 200 and rgb[2] < 200:
                    scene_score += 2 
                elif rgb[1] > 200 and rgb[0] < 200 and rgb[2] < 200:
                    scene_score += 3   
        if self.verbose > 0:
            print(f"Scene score: {scene_score}")      
        return scene_score
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    @staticmethod
    def get_image(C:ry.Config, cam_frame:str, verbose:int):
        camera_view = ry.CameraView(C)
        cam = C.getFrame(cam_frame)
        camera_view.setCamera(cam)
        img, _ = camera_view.computeImageAndDepth(C)
        img = np.asarray(img)

        if(verbose > 1):
            fig = plt.figure()
            fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
            plt.imshow(img)
            plt.show()
        return img
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def get_image(C:ry.Config, cam_frame:str, verbose):
        camera_view = ry.CameraView(C)
        cam = C.getFrame(cam_frame)
        camera_view.setCamera(cam)
        img, _ = camera_view.computeImageAndDepth(C)
        img = np.asarray(img)

        if(verbose > 1):
            fig = plt.figure()
            fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
            plt.imshow(img)
            plt.show()
        return img
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def display_solution(self):
        self.C.view(True, "Solution")
        for fs in self.FS:
            self.C.setFrameState(fs)
            self.C.view(False, "Solution")
            time.sleep(0.005)

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    @contextmanager
    def suppress_stdout(self):
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