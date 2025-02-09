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
from dataclasses import dataclass
import networkx as nx
import copy 
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import pandas as pd

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
        self.obstacle_pair_path = []
        self.obj_init_scene_score = {}
        self.obj_pts = {}
        self.moved_pos = []

    def run(self):
        tic_base = time.time()
        found = False
        wp_f = []
        L = 3
        path_found = False
        obs = []

        while not found:
            step_size = 0.03

            # Find a feasible pick configuration and path
            f, _ = self.find_pick_path(self.C, self.agent, self.obj, self.FS, self.verbose, wp_f=wp_f, K=4, N=1, step_size=step_size)
            
            # If it is not possible to go check if there is an obstacle that can be removed
            if not f: 
                fs0 = False
                # Remove obstacle
                if len(self.obs_list) > 0:
                    fs0, _ = self.remove_obstacle(0)
                    
                if not fs0:
                    return
            
            self.C.setFrameState(self.FS[-1])
                       
            # If path is cannot be followed, try with smaller step size
            for l in range(L):
                # Check for object path
                if not path_found:
                    if l == L-1:
                        self.C2 = self.make_agent(self.C, self.obj, self.agent, obs)
                    else: 
                        self.C2 = self.make_agent(self.C, self.obj, self.agent, [])

                    fr, P = self.run_rrt(self.C2, self.goal, [], self.verbose, N=3, step_size=step_size/(l+1), isOpt=True)

                    # If it is not possible to go check if there is an obstacle that can be removed
                    if not fr: 
                        fs1 = False
                        # Remove obstacle
                        if len(self.obs_list) > 0:
                            fs1, P = self.remove_obstacle(1)
                            self.C.setFrameState(self.FS[-1])
                            path_found = True
                            
                        if not fs1:
                            return
                        else:
                            break

                # Follow the RRT path with KOMO
                found, _, obs = self.solve_path(self.C, P, self.agent, self.obj, self.FS, self.verbose, cfc_lim=3,  K=2)
                if found:
                    break
                else:
                    path_found = False

        if found:
            print("Solution found!")
            tac_end = time.time()
            print("Total Time: ", tac_end - tic_base)
            return True
        else:
            print("Solution not found!")
            return False

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def remove_obstacle(self, type:int):

        self.OP = []
        self.obstacle_pair_path = {}
        self.obj_init_scene_score = {}
        self.obj_pts = {}
        self.moved_pos = []
        self.tried_pairs = []
        self.included_pairs = []
        pair_gen_time = 0

        # Type 0: Agent cannot reach obj, Type: 1 Obj cannot reach goal
        tic_base = time.time()
        # Generate obstacle pair
        self.obstacle_pairs = self.generate_obs_pairs()

        max_iter = 500
        idx = 0
        N = []

        prev_node = None
        isFirst = True
        while idx < max_iter:
            idx+=1

            if isFirst:
                self.find_collision_pairs(type, self.obstacle_pairs, N=2)
                if len(self.obstacle_pair_path) <= 0:
                    return False, None
                
                self.OP = self.weight_collision_pairs(self.obstacle_pair_path)

                tac_pair = time.time()
                pair_gen_time += tac_pair-tic_base
                print("Time for pair generation: ", pair_gen_time)

                for pair in self.OP:
                    C_node = ry.Config()
                    C_node.addConfigurationCopy(self.C)
                    root_node = Node(C_node, pair=pair.objects, C_hm=self.C_hm, isFirst=True)
                    N.append(root_node)

            # Select the best node
            node, isFirst = self.select_node(N, prev_node=prev_node, obs_path=self.obstacle_pair_path, isFirst=isFirst)

            if self.verbose > 0:
                print("Selected Node: " , node)
                if self.verbose > 1:
                    node.C.view(True, str(node))
                    node.C.view_close()
            
            node.visit += 1
            if prev_node != None:
                prev_node_id = prev_node.id
            else:
                prev_node_id = -1

            # Check if configuration is feasible
            f = False
            if type==0 and node.id != prev_node_id:

                if self.verbose > 0:
                    print("Checking if configuration is feasible")

                f, _ = self.find_pick_path(node.C, self.agent, self.obj, node.FS, self.verbose, K=2, N=1)    
                if f:
                    tac_coll = time.time()
                    print("SeGMan Time: ", tac_coll - tic_base)
                    self.FS.extend(node.FS)
                    return True, _
                
            elif type==1 and node.id != prev_node_id:
                C2 = self.make_agent(node.C, self.obj, self.agent)
                f, path = self.run_rrt(C2, self.goal, [], self.verbose, N=1, step_size=0.05, isOpt=True)
                if f:
                    tac_coll = time.time()
                    print("SeGMan Time: ", tac_coll - tic_base)
                    self.FS.extend(node.FS)
                    
                    return True, path     
            
            prev_node = node

            # Check which objects are reachable in the pair
            any_reach = False
            for o in node.pair:
                is_reach, reach_P = self.is_reachable(node, o)
                if not is_reach:  
                    continue

                any_reach = True
                node.moved_obj = o

                if self.verbose > 0:
                    print(f"Object {o} is REACHABLE")

                # For the reachable object, generate subgoals
                self.generate_subnode(node, o, N, P=reach_P, sample_count=3, radius=0.8, pair=node.pair)
           
            if not any_reach:
                if self.verbose > 0:
                    print("Node REMOVED: ", node.pair)
                N.remove(node)
                prev_node = None

        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def weight_collision_pairs(self, pair_path:dict): 

        # Extract keys and values
        path_names = list(pair_path.keys())
        path_values = list(pair_path.values())

        # Compute pairwise DTW distances
        n = len(path_values)
        distance_matrix2 = np.zeros((n, n))
        for i in range(n):
            for j in range(i, n):
                # Ensure both sequences remain 2D
                seq1 = np.array(path_values[i])  # Shape (m, 2)
                seq2 = np.array(path_values[j])  # Shape (m, 2)
                # Compute FastDTW distance using Euclidean metric
                dist, _ = fastdtw(seq1, seq2, dist=euclidean)
                distance_matrix2[i, j] = dist
                distance_matrix2[j, i] = dist  # Symmetric matrix
        # Normalize the matrix (Min-Max normalization)
        min_val = np.min(distance_matrix2[np.nonzero(distance_matrix2)])  # Ignore zeros
        max_val = np.max(distance_matrix2)
        if max_val > min_val:
            distance_matrix2 = (distance_matrix2 - min_val) / (max_val - min_val)

        if self.verbose > 1:
            print(path_names)
            df = pd.DataFrame(distance_matrix2)
            print(df)

        # Build the directed similarity graph
        directed_graph = nx.DiGraph()
        for i in range(n):
            directed_graph.add_node(path_names[i])
            distances = distance_matrix2[i]
            distances[i] = np.inf  # Exclude self-distance

            for j, dist in enumerate(distances):
                # 0.25 is a threshold for similarity
                if dist < 0.25:
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
            normalized_cluster_size = math.sqrt(cluster_size / max_cluster_size)

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
                if self.verbose > 0:
                    print(f"Pair: {name}, Cluster Size: {cluster_size}, Core Score: {core_score}, Pair Size: {pair_size},"
                           f"Num References: {num_references}, Pair Score: {pair_score}, Score: {final_score:.4f}")

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
                print(f"Pair: {pair.objects}, Weight: {pair.weight:.4f}")

        return weighted_obstacle_pairs

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def select_node(self, N:list, prev_node:Node, obs_path:list=[], isFirst:bool=False):
        if self.verbose > 0:
            print("Selecting the node")

        best_score = float('-inf')
        best_node = None

        for node in N:
            if node.isFirst:
                self.node_score(node, obs_path)

            elif prev_node != None and node.pair == prev_node.pair:
                self.node_score(node)
            
            if node.total_score > best_score:
                best_node = node
                best_score = node.total_score
    
        self.tried_pairs.append(best_node.pair)
        if sorted(self.tried_pairs) == sorted(self.included_pairs):
            return best_node, True

        return best_node, False
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def node_score(self, node:Node, obs_path:list=[]):
        c0 = 5
        gamma = 1
        weight_discount = 0.95

        for o in node.pair:
            node.C_hm.frame(o).setPosition(node.C.frame(o).getPosition())
        
        visit = node.visit
        parent_visit = 1

        global_scene_score = 0
        temporal_scene_score = 0
        
        node_weight = 1
        cur_pair = None
        for p in self.OP:
            if p.objects == node.pair:
                cur_pair = p
                node_weight = p.weight

        if node.isFirst:
            for i, p in enumerate(obs_path[tuple(node.pair)]):
                if i == 0:
                    obj = node.C_hm.addFrame("clone_"+str(i), "egoJoint", "shape:ssCylinder, Q:[-1.3 -1.3 0], size:[.3 .3 .02], color:[0 0 1]")
                else: 
                    obj = node.C_hm.addFrame("clone_"+str(i), "egoJoint", "shape:ssCylinder, Q:[-1.3 -1.3 0], size:[.2 .2 .02], color:[0 0 1]")
                obj.setPosition([*p, 0.1])

        if self.verbose > 1:
            node.C_hm.view(True, "Changed heatmap")

        for o in node.pair:
            # The node is root node
            if node.isFirst:
                scene_score, pts = self.scene_score(node.C_hm, o + "_cam_g", self.verbose)
                node.pts = pts
                self.obj_pts[tuple(node.pair)] = pts                   
                node.init_scene_scores[o] = scene_score
                node.prev_scene_scores[o] = scene_score
                global_scene_score += scene_score
                temporal_scene_score += 0
                
            elif node.total_score == float('-inf'):
                scene_score, pts = self.scene_score(node.C_hm, o + "_cam_g", self.verbose)
                node.pts = pts
                parent_visit = node.parent.visit
                global_scene_score += scene_score #- node.init_scene_scores[o]
                temporal_scene_score += scene_score - node.prev_scene_scores[o]
                node.prev_scene_scores[o] = scene_score
        
        node.isFirst = False

        global_scene_score /= len(node.pair)
        global_scene_score *= 0.006

        temporal_scene_score /= len(node.pair)
        temporal_scene_score *= 0.006

        # Just adjusting the visit count
        if node.total_score != float('-inf'):
            global_scene_score = node.global_scene_score
            temporal_scene_score = node.temporal_scene_score
            if node.parent != None:
                parent_visit = node.parent.visit
            visit = node.visit
            if cur_pair != None:
                cur_pair.weight *= weight_discount
        else:
            node.global_scene_score = global_scene_score
            node.temporal_scene_score = temporal_scene_score

        weighted_scene_score = ((1-gamma) * temporal_scene_score + gamma * global_scene_score) 
    
        exploitation = weighted_scene_score * math.sqrt(node_weight) * node.multiplier
        exploration  = c0 * math.sqrt(math.log(1+parent_visit) / visit)
        total_node_score = exploitation + exploration
        
        node.total_score = total_node_score
        
        if self.verbose > 0:
            print("Scored Node: ", node)
            if self.verbose > 0:
                print(f"Multiplier: {node.multiplier}, Global Scene Score: ", global_scene_score, "Temporal Scene Score: ", temporal_scene_score)
                print(f"Exploitation: {exploitation}, Exploration: {exploration}, Total Score: {total_node_score}")
            

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def make_agent(self, C:ry.Config, obj:str, agent:str="", obs:list=[]):
        C2 = ry.Config()
        C2.addConfigurationCopy(C)

        C2.delFrame(agent)
        C2.delFrame(agent+"Joint")
        
        obj_size   = C.frame(obj).getSize()
        pos = C.frame(obj).getPosition()     

        C2.delFrame(obj)
        obj_f = C2.addFrame(obj, obj+"Joint", "shape:ssBox, size:"+ str(obj_size) +", color:[0 0 1], contact:1")
        obj_J = C2.getFrame(obj+"Joint")

           
        obj_f.setJoint(ry.JT.transXY, limits=[-4, 4, -4, 4])
        obj_J.setPosition([0, 0, 0.2])
        C2.setJointState(pos[0:2])

        if len(obs) > 0:
            C2.addFrame("obs", "world", "shape:ssBox, size:"+ str(obj_size) +", color:[1 0 0], contact:1").setPosition([*obs[0:2], 0.2])
            #
        if agent != "":
            agent_size = C.frame(agent).getSize()
            obj_size   = C.frame(obj).getSize()
            if max(obj_size[0], obj_size[1]) < agent_size[0]:
                obj_f.setShape(ry.ST.ssBox, size=[agent_size[0]*1, agent_size[0]*1, .2, .02])

        return C2
   
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def generate_subnode(self, node:Node, o:str, N:list, P:list=[], sample_count:int = 30, radius:int=0.3, pair:list=[]):
        
        if self.verbose > 0:
            print("Generate subgoals")

        Ct = ry.Config()
        Ct.addConfigurationCopy(node.C)

        obj_pos = node.C.frame(o).getPosition()
        feas_count = 0
        err_count = 0
        poss = [Ct.frame(o).getPosition()]

        if len(self.moved_pos) == 0:
            for n in N:
                if n.pair == node.pair:
                    self.moved_pos.append(n.C.frame(o).getPosition())

        base_state = Ct.getFrameState()

        fs_base = copy.deepcopy(node.FS)
        if len(P) > 0:
            fs_base.extend(P)
            base_state = P[-1]
        
        # Filtering condition
        pts = copy.deepcopy(node.pts)

        generated_nodes = []

        for i in range(len(pts)):
            Ct.setFrameState(base_state)
            if len(node.pts) == 0:
                print("NOOO POINTS")
                return 
            
            obj_pos_n = pts[i] #random.choice(selected_points)
            
            if self.verbose > 0:
                print(f"Trying for {i} / {len(pts)}, Error: {err_count}", f"Subnode: {feas_count} / {sample_count}" )

            obj_mov_pos = [obj_pos_n[0], obj_pos_n[1], obj_pos[2]]
            
            Ct.addFrame("subgoal", "world", "shape: marker, size: [0.1]").setPosition(obj_mov_pos)  

            komo = ry.KOMO(Ct, phases=2, slicesPerPhase=15, kOrder=2, enableCollisions=True)                            # Initialize LGP
            komo.addControlObjective([], 1, 1e-1)
            komo.addControlObjective([], 2, 1e-1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)                                                                                        # Randomize the initial configuration
            komo.addObjective([1], ry.FS.distance, [self.agent, o], ry.OT.eq, scale=1e2, target=0)                                     # Pick
            komo.addModeSwitch([1,2], ry.SY.stable, [self.agent, o], True)   
            komo.addObjective([2] , ry.FS.positionDiff, [o, "subgoal"], ry.OT.sos, scale=1e1)  # Place constraints 
            
            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 

            Ct.delFrame("subgoal")

            if ret.feasible:

                path_frames=komo.getPathFrames()

                fs = copy.deepcopy(fs_base)
                fs.extend(path_frames)
                Ct.setFrameState(path_frames[-1])

                poss.append(Ct.frame(o).getPosition())

                new_node = Node(C=Ct, pair=node.pair, C_hm=node.C_hm, parent=node, layer=node.layer+1, FS=fs, init_scene_scores=node.init_scene_scores, prev_scene_scores=node.prev_scene_scores, moved_obj=o)

                multiplier = 1
                
                for ob in node.pair:
                    if ob != o:
                        #print("Checking for ", ob)
                        S2 = ry.Skeleton()
                        S2.enableAccumulatedCollisions(True)
                        S2.addEntry([0.5, -1], ry.SY.touch, [self.agent ,ob])
                        komo2 = S2.getKomo_path(Ct, 30, 1e0, 1e-1, 1e-1, 1e2)                                 
                        ret2 = ry.NLP_Solver(komo2.nlp(), verbose=0).solve() 
                        
                        if ret2.feasible:                              
                            multiplier *= 2

                new_node.multiplier = multiplier
                self.node_score(new_node)

                generated_nodes.append(new_node)
                feas_count += 1

        generated_nodes = sorted(generated_nodes, key=lambda n: n.total_score, reverse=True)
        N.extend(generated_nodes[:sample_count])

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def is_reachable(self, node:Node, o:str):
        Ct = ry.Config()
        Ct.addConfigurationCopy(node.C)

        if self.verbose > 0:
            print(f"Checking if object {o} is reachable")
        P = []

        if node.moved_obj == o:
            return True, P
        
        is_reachable, _ = self.find_pick_path(node.C, self.agent, o, FS=P, verbose=self.verbose, K=2, N=1)  
        return is_reachable, P

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
    def find_collision_pairs(self, type:int, obstacle_pairs:list, N:int=2):
        if self.verbose > 0:
            print("Finding collision pairs")
        
        obs_count = 0
        obj_goal = self.C.frame(self.obj).getPosition()[0:2]
        obs_pair = copy.deepcopy(obstacle_pairs)

        for _, op in enumerate(obs_pair):
            Ct = ry.Config()
            if type == 0:
                Ct.addConfigurationCopy(self.C)
                Ct.frame(self.obj).setContact(0)
            else:
                Ct.addConfigurationCopy(self.C2)

            path = []

            if self.verbose > 0:
                print(f"Trying: {op}")
            
            for o in op:
                Ct.frame(o).setContact(0)

            f = False
            if type == 0:
                f, path = self.run_rrt(Ct, obj_goal, [], self.verbose, N=1, step_size=0.05)
            elif type == 1:
                f, path = self.run_rrt(Ct, self.goal, [], self.verbose, N=1, step_size=0.05)
            
            if f:
                self.obstacle_pair_path[tuple(op)] = np.round(path, 2)
                self.included_pairs.append(op)
                obs_count +=1

            if self.verbose > 1:
                print(f"Is {op} blocking path: {f}")

            obstacle_pairs.remove(op)

            if obs_count >= N:
                break
            
        if self.verbose > 0:
            print(f"The blocking obstacle pairs: {self.obstacle_pair_path.keys()}")
                
        return
   
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_best_match(self, op:list, pair_clusters:dict):
        target_set = set(op)
        best_match = None
        max_common = 0
        max_overlap = 0.0 

        for arr in pair_clusters.keys():
            arr_set = set(arr)
            common_elements = target_set & arr_set
            common_count = len(common_elements)
            overlap = common_count / len(target_set) 

            if common_count > max_common or (common_count == max_common and overlap > max_overlap):
                best_match = arr
                max_common = common_count
                max_overlap = overlap

        return best_match

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_pick_path(self, C:ry.Config, agent:str, obj:str, FS:list, verbose: int, wp_f:list=[], K:int=5, N:int=5, step_size:float=0.03):

        if verbose > 0:
            print(f"Running Pick Path")

        for k in range(0, K):
            Ct = ry.Config()
            Ct.addConfigurationCopy(C)
            if verbose > 1:
                print(f"Trying Pick KOMO for {k}")

            S = ry.Skeleton()
            S.enableAccumulatedCollisions(True)
            S.addEntry([0, 1], ry.SY.touch, [agent ,obj])
            komo = S.getKomo_path(Ct, 1, 1e0, 1e-1, 1e-1, 1e2)                                   # Pick
            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()

            if k != 0: 
                komo.initRandom()  

            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 

            if verbose > 1:
                komo.view_play(True, f"Pick Komo Solution: {ret.feasible}, {ret.eq < 1}")
                komo.view_close()
                
            if ret.feasible:
                fr, P = self.run_rrt(C, komo.getPath()[-1], FS, verbose, N, step_size=step_size, isOpt=True)
                if fr:
                    return True, P
        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def run_rrt(self, C:ry.Config, goal:list, FS:list, verbose: int, N:int=20, step_size:float=0.05, isOpt:bool=False):
        Ct = ry.Config()
        Ct.addConfigurationCopy(C)
        if verbose > 1:
            js = Ct.getJointState()
            Ct.view(True, "RRT Init")
            Ct.setJointState(goal)
            Ct.view(True, "RRT Final")
            Ct.view_close()
            Ct.setJointState(js)
        for n in range(N):
            # Find feasible path between configurations
            step_size /= (n+1)
            if verbose > 1:
                print(f"Trying RRT for {n}")
            with self.suppress_stdout():
                ry.params_clear()
                ry.params_add({"rrt/stepsize": step_size})
                rrt = ry.PathFinder()                    
                rrt.setProblem(Ct, Ct.getJointState(), goal)
                s = rrt.solve()
                ry.params_clear()
                ry.params_add({"rrt/stepsize": 0.01})
            if s.feasible:
                path = s.x
                if isOpt:
                    komo = ry.KOMO(Ct, phases=len(path)/10, slicesPerPhase=10, kOrder=2, enableCollisions=True)                           
                    komo.addControlObjective([], 1, 1e-1)
                    komo.addControlObjective([], 2, 1e-1)
                    komo.initWithWaypoints(path, 10)                                                                                 # Randomize the initial configuration
                    ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
                    path = komo.getPath()
                if verbose > 1:
                    Ct.view(True, "RRT Solution")
                for p in path:
                    Ct.setJointState(p)
                    FS.extend([Ct.getFrameState()])
                    if verbose > 1:
                        Ct.view(False, "RRT Solution")
                        time.sleep(0.01)
                Ct.view_close()
                return True, path
        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def solve_path(self, C:ry.Config, P:list, agent:str, obj:str, FS:list, verbose: int, cfc_lim:int=5, K:int=5):
        Ct = ry.Config()
        Ct.addConfigurationCopy(C)

        if self.verbose > 0:
            print("Solving for path")

        max_step = len(P) - 1
        step = len(P)-1
        pi = step

        cfc = 0 # consequtive feas count
        fs = []
        pf = 0

        while pi < len(P):
            wp = P[pi]
            feasible = False

            for k in range(0, K+1):

                if verbose > 1:
                    print(f"pi:{pi}, tot: {len(P)-1}")
                    print("Step:", step)

                if verbose > 1:
                    print(f"Trying Move KOMO for {k+1} time")

                Ct.addFrame("subgoal", "world", "shape: marker, size: [0.1]").setPosition([*wp, 0.2])
                komo = ry.KOMO(Ct, phases=2, slicesPerPhase=20, kOrder=2, enableCollisions=True)   
                komo.addControlObjective([], 1, 1e-1)
                komo.addControlObjective([], 2, 1e-1)
                komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)                                                                                        # Randomize the initial configuration
                komo.addObjective([1], ry.FS.distance, [agent, obj], ry.OT.eq, scale=1e2, target=0)                                     # Pick
                komo.addModeSwitch([1,2], ry.SY.stable, [agent, obj], True)   
                komo.addObjective([2] , ry.FS.positionDiff, [obj, "subgoal"], ry.OT.eq, scale=1e0)  # Place constraints 

                if k > 1:
                    komo.initRandom()

                ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
                Ct.delFrame("subgoal")
                feasible = ret.feasible
                if ret.feasible:
                    Ct.setFrameState(komo.getPathFrames()[-1])
                    fs.extend(komo.getPathFrames())

                    if pi == len(P)-1:
                        FS.extend(fs)
                        return True, Ct, None
                    
                    if cfc >= cfc_lim:
                        step = min(max_step, step*2)

                    pf = pi
                    pi = min(pf + step, len(P)-1)
                    cfc += 1
                    break
                
            if not feasible: 
                cfc = 0
                if step == 1:
                    return False, Ct, Ct.frame(obj).getPosition()
                step = int(step / 2)
                pi = min(pf + step, len(P)-1)

        FS.extend(fs)
        return True, Ct, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def scene_score(self, C:ry.Config, cam_frame:str, verbose:int=0):
        camera_view = ry.CameraView(C)
        cam = C.getFrame(cam_frame)
        camera_view.setCamera(cam)
        img, depth = camera_view.computeImageAndDepth(C)

        img = np.asarray(img)

        img_mask = copy.deepcopy(img)

        scene_score = 0
        for i, r in enumerate(img):
            for j, rgb in enumerate(r):
                if rgb[1] > 150 and rgb[0] < 150 and rgb[2] < 150:
                    scene_score += 2 
                elif rgb[0] < 150 and rgb[1] < 150 and rgb[2] > 150:
                    scene_score += 3  
                else:
                    depth[i][j] = 0
                    img_mask[i][j] = [0, 0, 0]
        
        if(self.verbose > 1):
            fig = plt.figure()
            fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
            plt.imshow(img_mask)
            plt.show()

            fig = plt.figure()
            fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
            plt.imshow(img)
            plt.show()


        pts = ry.depthImage2PointCloud(depth, camera_view.getFxycxy())
        pts = self.cam_to_target(pts.reshape(-1, 3), C.getFrame("world"), cam)
        random.shuffle(pts)
        pts = pts[::8]

        if(verbose>1):
            C2 = ry.Config()
            C2.addConfigurationCopy(C)
            C3 = ry.Config()
            C3.addConfigurationCopy(C)

            C2.getFrame("world").setPointCloud(pts, [0,0,0])
            C2.view(True,"1")

            C3.getFrame("world").setPointCloud(pts, [0,0,0])
            C3.view(True,"2")

        if self.verbose > 1:
            print(f"Scene score: {scene_score}")  
        
        return scene_score, pts

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #


    @staticmethod
    def cam_to_target(pts, cam_frame, target_frame):
        pts = pts[~np.all(pts == 0, axis=1)]  # Filter out rows where all values are 0

        if pts.shape[0] == 0:
            return np.empty((0, 3)) 
        
        # Camera position and rotation (in world frame)
        t_cam_world = cam_frame.getPosition()  # (3, )
        R_cam_world = cam_frame.getRotationMatrix()  # (3, 3)

        # Target frame position and rotation (in world frame)
        t_target_world = target_frame.getPosition()  # (3, )
        R_target_world = target_frame.getRotationMatrix()  # (3, 3)

        # Compute the transformation from camera frame to target frame
        R_target_cam = np.dot(R_target_world, R_cam_world.T)  # (3,3) Relative rotation
        t_target_cam = t_target_world - np.dot(R_target_world, t_cam_world)  # (3, )

        # Convert points to homogeneous coordinates
        points_camera_frame_homogeneous = np.hstack((pts, np.ones((pts.shape[0], 1))))  # (N,4)

        # Construct transformation matrix (4x4)
        transformation_matrix = np.vstack((
            np.hstack((R_target_cam, t_target_cam.reshape(-1, 1))), 
            np.array([0, 0, 0, 1])
        ))  # (4,4)

        # Apply transformation
        points_target_frame_homogeneous = np.dot(transformation_matrix, points_camera_frame_homogeneous.T).T

        # Extract the transformed points
        points_target_frame = points_target_frame_homogeneous[:, :3]

        return points_target_frame
           
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def display_solution(self, FS:list=None, pause:float=0.05):
        Ct = ry.Config()
        Ct.addConfigurationCopy(self.C)

        if FS == None:
            FS = self.FS

        Ct.view(True, "Solution")
        for fs in FS:
            Ct.setFrameState(fs)
            Ct.view(False, "Solution")
            time.sleep(pause)
        Ct.view(True, "Solution")

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