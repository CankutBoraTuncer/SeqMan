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
import numpy as np
from frechetdist import frdist
from dataclasses import dataclass
import networkx as nx
from dtaidistance import dtw
import copy 

@dataclass
class Pair:
    objects: list
    weight: float

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
            f, _ = self.find_pick_path(self.C, self.agent, self.obj, self.FS, self.verbose, 1, 1)
            
            # If it is not possible to go check if there is an obstacle that can be removed
            if not f: 
                # Remove obstacle
                fs, FS = self.remove_obstacle(0)
                if not fs:
                    return
                else:
                    self.FS.append(FS)
            
            self.C.setFrameState(self.FS[-1])
            self.C.view(True, "True")

            # Check for object path
            C2.setJointState(self.C.frame(self.obj).getPosition()[0:2])
            P, fr = self.find_place_path(C2, self.goal, self.verbose)

            # If it is not possible to go check if there is an obstacle that can be removed
            if not fr: 
                # Remove obstacle
                fs = self.remove_obstacle(1)
                if not fs:
                    return
                else:
                    continue

            # Follow the RRT path with KOMO
            found, self.C = self.solve_path(self.C, P, self.agent, self.obj, self.FS, self.verbose)

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
        obstacle_pair = self.generate_obs_pairs()

        # Check which pairs are not relevant with the source of failure
        obstacle_pair_path = self.find_collision_pairs(type, obstacle_pair)

        # Cluster the pairs based on the path similarity
        self.OP = self.weight_collision_pairs(obstacle_pair_path)
        
        max_iter = 500
        idx = 0
        N = []
        for pair in self.OP:
            C_node = ry.Config()
            C_node.addConfigurationCopy(self.C)
            root_node = Node(C_node, pair=pair)
            N.append(root_node)
        
        prev_node = None
        isFirst = True
        while len(N) > 0 and idx < max_iter:
            idx+=1

            # Select the best node
            node = self.select_node(N, prev_node=prev_node, isFirst=isFirst)
            isFirst = False

            if self.verbose > 1:
                print("Selected Node: " , node)
                node.C.view(True, str(node))
                node.C.view_close()

            node.visit += 1
            prev_node = node

            # Check if configuration is feasible
            f = False
            if type==0:
                if self.verbose > 0:
                    print("Checking if configuration is feasible")
                f, _ = self.find_pick_path(node.C, self.agent, self.obj, node.FS, 2, K=1, N=1)
                if f:
                    self.display_solution(node.FS)
                    return True, node.FS 
            else:
                f, P  = self.find_place_path(node.C, node.C.frame(self.obj).getPosition()[0:2], self.verbose, N=2)
                if f:
                    return True, P     
            
            # Check which objects are reachable in the pair
            any_reach = False
            for o in node.pair.objects:
                if not self.is_reachable(node, o):
                    continue
                any_reach = True
                print("Trying OBJECT: ", o)

                # For the reachable object, generate subgoals
                Z = self.generate_subgoal(node, o, sample_count=10)

                # For each subgoal try to pick and place
                C2 = self.make_agent(node.C, o)
                P, f1 = self.find_place_path(C2, z, self.verbose, N=3)
                if f1:
                    fs = copy.deepcopy(node.FS)
                    for z in Z:
                        feas, C_n = self.solve_path(node.C, P, self.agent, o, fs, self.verbose, K=2)
                        if feas and not self.reject(N, C_n, node.pair):
                            # Calculate the scene score
                            new_node = Node(C=C_n, pair=node.pair, parent=node, layer=node.layer+1, FS=fs, init_scene_scores=node.init_scene_scores, prev_scene_scores=node.prev_scene_scores)
                            self.node_score(new_node)

                            if self.verbose > 1:
                                C_n.view(True, "New Node")
                            N.append(new_node)
                            
            if not any_reach:
                if self.verbose > 0:
                    print("Node REMOVED: ", node.pair.objects)
                N.remove(node)
                prev_node = None

        return False

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def reject(self, N:list, C:ry.Config, pair:Pair):

        obs_pos = {}
        for obs in pair.objects:
            obs_pos[obs] = C.frame(obs).getPosition()[0:2]
        
        for node in N:
            if node.pair == pair:
                    sim_count = 0
                    for obs in pair.objects:
                        if np.linalg.norm(obs_pos[obs] - node.C.frame(obs).getPosition()[0:2]) < 0.10:
                            sim_count += 1
                    if sim_count == len(pair.objects):
                        if self.verbose > -1:
                            print("Node REJECTED: ", pair.objects)
                        return True
        return False
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def weight_collision_pairs(self, pair_path:dict): 

        # Make all arrays the same length
        min_length = min(len(value) for value in pair_path.values())
        pair_path = {key: value[:min_length] for key, value in pair_path.items()}

        # Extract keys and values
        path_names = list(pair_path.keys())
        path_values = list(pair_path.values())

        # Compute pairwise Fréchet distances
        n = len(path_values)
        distance_matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(i + 1, n):
                distance = frdist(path_values[i], path_values[j])
                distance_matrix[i, j] = distance
                distance_matrix[j, i] = distance

        # Compute pairwise DTW distances
        n = len(path_values)
        distance_matrix2 = np.zeros((n, n))
        for i in range(n):
            for j in range(i, n):
                distance = dtw.distance(path_values[i].flatten(), path_values[j].flatten())
                distance_matrix2[i, j] = distance
                distance_matrix2[j, i] = distance

        # Build the directed similarity graph
        directed_graph = nx.DiGraph()
        for i in range(n):
            distances = distance_matrix2[i]
            distances[i] = np.inf  # Exclude self-distance
            closest_idx = np.argmin(distances)
            directed_graph.add_edge(path_names[i], path_names[closest_idx])
            for j, dist in enumerate(distances):
                if dist < 0.2:
                    directed_graph.add_edge(path_names[i], path_names[j])

        # Find weakly connected components (clusters disregarding direction)
        clusters = list(nx.weakly_connected_components(directed_graph))

        # Calculate cluster sizes
        cluster_sizes = [
            sum(len(key) for key in cluster) for cluster in clusters
        ]
        max_cluster_size = max(cluster_sizes)

        # Group paths by cluster and compute scores
        weighted_obstacle_pairs = []
        for cluster, cluster_size in zip(clusters, cluster_sizes):
            # Normalize cluster size
            normalized_cluster_size = cluster_size / max_cluster_size

            # Find the smallest pair in the cluster (the core pair which exists in all pairs)
            core_pair = min(cluster, key=len)

            # Calculate pair scores and create Pair objects
            for name in cluster:
                pair_size = len(name)
                num_references = directed_graph.in_degree(name)  # Count incoming edges
                pair_score = 1 / pair_size
                core_score = 2 if core_pair == name else 1
                final_score = core_score * pair_score * normalized_cluster_size
                weighted_obstacle_pairs.append(Pair(objects=[*name], weight=final_score)) 
                if self.verbose > 1:
                    print(f"Pair: {name}, Cluster Size: {cluster_size}, Pair Size: {pair_size}, Num References: {num_references}, Pair Score: {pair_score}, Score: {final_score:.4f}")

        weighted_obstacle_pairs = sorted(weighted_obstacle_pairs, key=lambda pair: pair.weight, reverse=True)

        if self.verbose > 1:
            # Create and visualize the directed graph using networkx
            plt.figure(figsize=(12, 8))
            pos = nx.spring_layout(directed_graph)  # Layout for better visualization
            nx.draw(
                directed_graph, pos, with_labels=True, node_color='lightblue', edge_color='gray',
                node_size=2000, font_size=10, font_weight='bold', arrows=True
            )
            nx.draw_networkx_edges(
                directed_graph, pos, edge_color='red', arrowstyle='-|>', arrowsize=20
            )
            plt.title("Directed Path Similarity Graph")
            plt.show()

        if self.verbose > 0:
            # Output Pair objects
            for pair in weighted_obstacle_pairs:
                print(f"Pair: {pair.objects}, Score: {pair.weight:.4f}")

        return weighted_obstacle_pairs

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def select_node(self, N:list, prev_node:Node, isFirst:bool=False):
        if self.verbose > 0:
            print("Selecting the node")

        best_score = float('-inf')
        best_node = None

        for node in N:
            if isFirst:
                self.node_score(node)
            elif node == prev_node:
                self.node_score(node)
                
            if node.total_score > best_score:
                best_node = node
                best_score = node.total_score
            
        #if prev_node != None and all(element in best_node.pair for element in prev_node.pair):
        #    N.append(Node(prev_node.C, best_node.pair, layer=best_node.layer, FS=prev_node.FS, score=best_node.score, is_reachable=best_node.is_reachable))
        #    prev.C.view(True, "New Node")

        return best_node
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def node_score(self, node:Node):
        Ct = ry.Config()
        Ct.addConfigurationCopy(self.C_hm)

        c0 = 5
        c1 = 1
        gamma = 0.3

        for o in node.pair.objects:
            Ct.frame(o).setPosition(node.C.frame(o).getPosition())
            
        if self.verbose > 1:
            Ct.view(True, "Changed heatmap")

        visit = node.visit
        parent_visit = 1

        global_scene_score = 0
        temporal_scene_score = 0
        
        for o in node.pair.objects:
            # The node is root node
            if node.parent == None:
                gamma = 5e-2
                scene_score = self.scene_score(Ct, o + "_cam_g")
                node.init_scene_scores[o] = scene_score
                node.prev_scene_scores[o] = scene_score
                global_scene_score += node.init_scene_scores[o]
                temporal_scene_score += 0
            else:
                # New node
                if node.total_score == float('-inf'):
                    scene_score = self.scene_score(Ct, o + "_cam_g")
                    parent_visit = node.parent.visit
                    global_scene_score += scene_score - node.init_scene_scores[o]
                    temporal_scene_score += scene_score - node.prev_scene_scores[o]
                    node.prev_scene_scores[o] = scene_score

        if node.total_score != float('-inf'):
            global_scene_score = node.global_scene_score
            temporal_scene_score = node.temporal_scene_score
        else:
            node.global_scene_score = global_scene_score
            node.temporal_scene_score = temporal_scene_score

        weighted_scene_score = ((1-gamma) * temporal_scene_score + gamma * global_scene_score) / (30*30*3*c1)
        exploitation = weighted_scene_score * node.pair.weight
        exploration  = c0 * math.sqrt(math.log(1+parent_visit) / visit)
        total_node_score = exploitation + exploration
        
        node.total_score = total_node_score
        
        if self.verbose > 0:
            print("Global Scene Score: ", global_scene_score, "Temporal Scene Score: ", temporal_scene_score)
            print(f"Exploitation: {exploitation}, Exploration: {exploration}, Total Score: {total_node_score}")
            print("Scored Node: ", node)


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

    def is_reachable(self, node:Node, o:str):
        if self.verbose > 0:
            print(f"Checking if object {o} is reachable")
        is_reachable, _ = self.find_pick_path(node.C, self.agent, o, [], self.verbose, K=3, N=1)  
        return is_reachable

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def generate_obs_pairs(self):
        OP = []
        if self.verbose > 0:
            print("Generating obstacle pair")
        for r in range(1, len(self.obs_list) + 1):
            OP.extend(combinations(self.obs_list, r))
        OP = [list(item) for item in OP]
        return OP

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
    def find_collision_pairs(self, type:int, OP:list):
        if self.verbose > 0:
            print("Finding collision pairs")
        
        goal = self.C.frame(self.obj).getPosition()[0:2]
        goal_set = False
        pair_path = {}

        for _, op in enumerate(OP):
            Ct = ry.Config()
            Ct.addConfigurationCopy(self.C)

            path = []

            if self.verbose > 0:
                print(f"Trying: {op}")
            
            for o in op:
                Ct.frame(o).setContact(0)

            f = False
            if type == 0:
                if not goal_set:
                    f, goal = self.find_pick_komo(Ct, self.agent, self.obj, self.verbose, K=3)
                    if f:
                        goal_set = True
                        f, path = self.run_rrt(Ct, goal, [], verbose=self.verbose, N=1)

                else:
                    f, path = self.run_rrt(Ct, goal, [], verbose=self.verbose, N=1)
            else:
                f, path = self.find_place_path(Ct, goal, self.verbose, N=3)

            if f:
                pair_path[tuple(op)] = path

            if self.verbose > 1:
                print(f"Is {op} blocking path: {f}")

        if self.verbose > 0:
            print(f"The blocking obstacle pairs: {pair_path.keys()}")
        return pair_path

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
        Ct = ry.Config()
        Ct.addConfigurationCopy(C)
        if verbose > 1:
            print(f"Running Pick KOMO")
        for k in range(K):
            # Find a feasible touch configuration
            if verbose > 1:
                print(f"Trying Pick KOMO for {k}")
            S = ry.Skeleton()
            S.enableAccumulatedCollisions(True)
            S.addEntry([0, 1], ry.SY.touch, [agent, obj])
            komo = S.getKomo_path(Ct, 1, 1e-1, 1e1, 1e-1, 1e2)
            if k != 1: 
                komo.initRandom()  
            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
            if verbose > 1:
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
                ry.params_add({"rrt/stepsize": 0.05})
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
                return True, path
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
                        time.sleep(0.1)
                    Ct.view_close()
                return path, True
        return None, False

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def solve_path(self, C:ry.Config, P:list, agent:str, obj:str, FS:list, verbose: int, K:int=5):
        Ct = ry.Config()
        Ct.addConfigurationCopy(C)

        if self.verbose > 0:
            print("Solving for path")
        for pi, wp in enumerate(P):
            if verbose > 1:
                print(f"{pi} / {len(P)-1}")
            for k in range(K):
                Ct.addFrame("subgoal", "world", "shape: marker, size: [0.1]").setPosition([*wp, 0.2])
                if verbose > 1:
                    print(f"Trying Move KOMO for {k}")
                S = ry.Skeleton()
                S.enableAccumulatedCollisions(True)
                S.addEntry([0.1, -1], ry.SY.touch, [agent, obj])
                S.addEntry([0.2, -1], ry.SY.stable, [agent, obj])
                S.addEntry([0.3, 0.4], ry.SY.positionEq, ["subgoal", obj])
                komo = S.getKomo_path(Ct, 10, 1e-5, 1e-3, 1e-5, 1e1)
                ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
                Ct.delFrame("subgoal")
                feasible = ret.eq < 1
                if verbose > 1 and not feasible:
                    komo.view_play(True, f"Move Komo Solution: {feasible}")
                    komo.view_close()
                if feasible:
                    Ct.setFrameState(komo.getPathFrames()[-1])
                    FS.append(komo.getPathFrames())
                    break
            if pi == len(P)-1:
                return True, Ct
        return False, None

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def scene_score(self, C:ry.Config, cam_frame:str):
        img = SeGMan.get_image(C, cam_frame, 2)
        scene_score = 0
        for r in img:
            for rgb in r:
                if rgb[0] > 200 and rgb[1] < 200 and rgb[2] < 200:
                    scene_score += 1
                elif rgb[1] > 200 and rgb[0] > 200 and rgb[2] < 200:
                    scene_score += 2 
                elif rgb[1] > 200 and rgb[0] < 200 and rgb[2] < 200:
                    scene_score += 3   
                elif rgb[0] < 200 and rgb[1] < 200 and rgb[2] > 200:
                    scene_score += 10  
        if self.verbose > 1:
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

    def display_solution(self, FS:list=None):
        Ct = ry.Config()
        Ct.addConfigurationCopy(self.C)

        if FS == None:
            FS = self.FS

        Ct.view(True, "Solution")
        for fs in FS:
            Ct.setFrameState(fs)
            Ct.view(False, "Solution")
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