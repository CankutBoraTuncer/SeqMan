import robotic as ry
import time
import os
from contextlib import contextmanager
from itertools import combinations
from src.Node import Node
import numpy as np
import matplotlib.pyplot as plt
import math
import random

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
            f, js = self.find_pick_path(self.C, self.agent, self.obj, self.FS, self.verbose, 1, 1)
            
            # If it is not possible to go check if there is an obstacle that can be removed
            if not f: 
                # Remove obstacle
                fs = self.remove_obstacle(0)
                if not fs:
                    return
            
            print(f, js)
            self.C.setJointState(js)

            # Check for object path
            C2.setJointState(self.C.frame(self.obj).getPosition()[0:2])
            P, fr = self.find_place_path(C2, self.goal, self.verbose)

            # If it is not possible to go check if there is an obstacle that can be removed
            if not fr: 
                # Remove obstacle
                fs = self.remove_obstacle(1)
                if not fs:
                    return
            
            # Follow the RRT path with KOMO
            found = self.solve_path(self.C, P, self.agent, self.obj, self.FS, self.verbose)

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
            N.append(Node(self.C, op, layer=1))
        
        while len(N) > 0 and idx < max_iter:
            idx+=1

            # Select the best node
            node = self.select_node(N)

            # Check if configuration is feasible
            f = False
            if type==0:
                f, _ = self.find_pick_path(node.C, self.agent, self.obj, node.FS, self.verbose, 2, 2)
                if f:
                    
                    return node.FS, True
            else:
                P, f = self.find_place_path(node.C, node.C.frame(self.obj).getPosition()[0:2], self.verbose, 2)
                if f:
                    return P, True    
            
            # Check which objects are reachable in the pair
            any_reach = False
            for o in node.op:
                if not self.is_reachable(node.C, o):
                    continue
                any_reach = True

                # For the reachable object, generate subgoals
                Z = self.generate_subgoal(node, o)

                # For each subgoal try to pick and place
                for z in Z:
                    C2 = self.make_agent(node.C, o)
                    P, f1 = self.find_place_path(C2, z, 3, 2)
                    if f1: 
                        feas = self.solve_path(self.C, P, self.agent, o, self.FS, 3)
                        if feas:
                            # Calculate the scene score
                            node_score = self.node_score(node, o)
                            N.append(Node(C=z.C, op=node.op, layer=node.layer+1, FS=node.FS, score = node_score))

            if not any_reach:
                N.remove(node)

        return False

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def node_score(self, node:Node, o:str):
        Ct = ry.Config()
        Ct.addConfigurationCopy(self.C_hm)

        frame_names = node.C.getFrameNames()
        for fn in frame_names:
            if "obj" in fn:
                Ct.frame(fn).setPosition(node.C.frame(fn).getPosition())
        
        if self.verbose > 1:
            Ct.view(True, "Changed heatmap")
        
        scene_score = self.scene_score(Ct, o + "_cam_rel")
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

        obj_f = C2.getFrame(obj)
        pos = obj_f.getPosition()
        obj_f.setPosition([0, 0, 0.2])

        obj_f.setJoint(ry.JT.transXY, limits=[-4, 4, -4, 4])
        C2.setJointState(pos[0:2])

        return C2
   
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def generate_subgoal(self, node:Node, o:str, sample_count:int = 30, radius:int=0.3):
        if self.verbose > 0:
            print("Generate subgoals")
        obj_pos = node.C.frame(o).getPosition()
        Z = []
        for _ in range(sample_count*5):
            Ct = ry.Config()
            Ct.addConfigurationCopy(node.C)
            col_pre = Ct.getCollisions()
            new_pos = obj_pos + [random.uniform(-radius, radius), random.uniform(-radius, radius), 0]
            Ct.frame(o).setPosition(new_pos)
            Ctt = ry.Config()
            Ctt.addConfigurationCopy(Ct)
            col_post = Ctt.getCollisions()

            if len(col_pre) == len(col_post):
                Z.append(new_pos[0:2])
            if len(Z) == sample_count:
                break

        return Z

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def is_reachable(self, C:ry.Config, o:str):
        if self.verbose > 0:
            print(f"Checking if object {o} is reachable")
        
        f, _ = self.find_pick_path(C, self.agent, o, [], 3, 2, 2)
        return f
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def select_node(self, N:list):
        if self.verbose > 0:
            print("Selecting the node")
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
        if self.verbose > 0:
            print("Generating obstacle pair")
        for r in range(1, len(self.obs_list) + 1):
            self.OP.extend(combinations(self.obs_list, r))
        self.OP = [list(item) for item in self.OP]

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
    def find_collision_pair(self, type:int):
        if self.verbose > 0:
            print("Finding collision pairs")
        
        goal = self.C.frame(self.obj).getPosition()[0:2]
        goal_set = False
        for _, op in enumerate(self.OP):
            Ct = ry.Config()
            Ct.addConfigurationCopy(self.C)
            if self.verbose > 1:
                print(f"Trying: {op}")
            for o in op:
                Ct.frame(o).setContact(0)
            f = False
            if type == 0:
                if not goal_set:
                    f, goal = self.find_pick_komo(Ct, self.agent, self.obj, self.verbose, K=2)
                    if f:
                        goal_set = True
                else:
                    f, _ = self.run_rrt(Ct, goal, [], self.verbose, N=2)
            else:
                f = self.find_place_path(Ct, goal, self.verbose, 2)
            if self.verbose > 1:
                print(f"Is {op} blocking path: {f}")
            if not f:
                self.OP.remove(op)
        print(f"The blocking obstacle pairs: {self.OP}")

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_pick_path(self, C:ry.Config, agent:str, obj:str, FS:list, verbose: int, K:int=20, N:int=20):
        if verbose > 0:
            print(f"Running Pick Path")
        feas, goal = self.find_pick_komo(C, agent, obj, verbose, K)
        if feas:
            return self.run_rrt(C, goal, FS, verbose, N)
        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_pick_komo(self, C:ry.Config, agent:str, obj:str, verbose: int, K:int=20):
        if verbose > 1:
            print(f"Running Pick KOMO")
        for k in range(K):
            # Find a feasible touch configuration
            if verbose > 1:
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
                return True, komo.getPath()[-1]    
        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def run_rrt(self, C:ry.Config, goal:list, FS:list, verbose: int, N:int=20):
        Ct = ry.Config()
        Ct.addConfigurationCopy(C)
        for n in range(N):
            # Find feasible path between configurations
            if verbose > 1:
                print(f"Trying Pick RRT for {n}")
            with self.suppress_stdout():
                ry.params_clear()
                ry.params_add({"rrt/stepsize": 0.1})
                rrt = ry.PathFinder()                    
                rrt.setProblem(Ct, Ct.getJointState(), goal)
                s = rrt.solve()
                ry.params_clear()
                ry.params_add({"rrt/stepsize": 0.01})
            if s.feasible:
                path = s.x
                if verbose > 1:
                    Ct.view(True, "Pick Solution")
                for p in path:
                    Ct.setJointState(p)
                    FS.append(Ct.getFrameState())
                    if verbose > 1:
                        Ct.view(False, "Pick Solution")
                        time.sleep(0.05)
                Ct.view_close()
                return True, path[-1]
        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_place_path(self, C:ry.Config, goal:list, verbose: int, N:int = 20):
        Ct = ry.Config()
        Ct.addConfigurationCopy(C)
        if verbose > 0:
            print(f"Running Place Path")
        for n in range(N):
            # Find RRT path for object
            if verbose > 1:
                print(f"Trying Place RRT for {n}")

            with self.suppress_stdout():
                rrt = ry.PathFinder()                    
                rrt.setProblem(Ct, Ct.getJointState(), goal)
                s = rrt.solve()
            if s.feasible:
                path = s.x
                if verbose > 1:
                    C.view(True, "Place Solution")
                    for p in path:
                        Ct.setJointState(p)
                        Ct.view(False, "Place Solution")
                        time.sleep(0.05)
                    Ct.view_close()
                return path, True
        return None, False

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def solve_path(self, C:ry.Config, P:list, agent:str, obj:str, FS:list, verbose: int):
        if self.verbose > 0:
            print("Solving for path")
        for pi, wp in enumerate(P):
            if self.verbose > 0:
                print(f"{pi} / {len(P)-1}")
            K = 5
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
                if verbose > 1 and feasible:
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

        if(verbose > -1):
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