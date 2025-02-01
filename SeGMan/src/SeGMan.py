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

        while not found:
            step_size = 0.03

            # Find a feasible pick configuration and path
            f, _ = self.find_pick_path(self.C, self.agent, self.obj, self.FS, self.verbose, wp_f=wp_f, K=1, N=1)
            
            # If it is not possible to go check if there is an obstacle that can be removed
            if not f: 
                fs = False
                # Remove obstacle
                if len(self.obs_list) > 0:
                    fs, _ = self.remove_obstacle(0)
                if not fs:
                    return
            
            self.C.setFrameState(self.FS[-1])

            # If path is cannot be followed, try with smaller step size
            for l in range(L):
                # Check for object path
                self.C2 = self.make_agent(self.C, self.obj)
                fr, P = self.run_rrt(self.C2, self.goal, [], self.verbose, N=3, step_size=step_size/(l+1))
                
                # If it is not possible to go check if there is an obstacle that can be removed
                if not fr: 
                    fs = False
                    # Remove obstacle
                    if len(self.obs_list) > 0:
                        fs, P = self.remove_obstacle(1)
                        self.C.setFrameState(self.FS[-1])
                    if not fs:
                        return

                # Follow the RRT path with KOMO
                found, self.C, wp_f = self.solve_path(self.C, P, self.agent, self.obj, self.FS, self.verbose, K=3)
                if found:
                    break

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
        self.obstacle_pair_path = []
        self.obj_init_scene_score = {}
        self.obj_pts = {}
        self.moved_pos = []

        # Type 0: Agent cannot reach obj, Type: 1 Obj cannot reach goal
        tic_base = time.time()
        # Generate obstacle pair
        self.obstacle_pairs = self.generate_obs_pairs()

        # Check which pairs are not relevant with the source of failure
        self.obstacle_pair_path = self.find_collision_pairs(type, self.obstacle_pairs)
        #pair_clusters = self.find_collision_pairs_v2(N=3)

        # Cluster the pairs based on the path similarity
        self.OP = self.weight_collision_pairs(self.obstacle_pair_path)
        #self.OP = self.weight_collision_pairs_v2(pair_clusters)
        
        tac_pair = time.time()
        print("Time for pair generation: ", tac_pair-tic_base)
        
        max_iter = 500
        idx = 0
        N = []
        for pair in self.OP:
            C_node = ry.Config()
            C_node.addConfigurationCopy(self.C)
            root_node = Node(C_node, pair=pair.objects, C_hm=self.C_hm)
            N.append(root_node)
        
        prev_node = None
        isFirst = True
        while len(N) > 0 and idx < max_iter:
            idx+=1

            # Select the best node
            node = self.select_node(N, prev_node=prev_node, obs_path=self.obstacle_pair_path, isFirst=isFirst)
            isFirst = False
            
            if self.verbose > 0:
                print("Selected Node: " , node)
                if self.verbose > 2:
                    node.C.view(True, str(node))
                    node.C.view_close()

            node.visit += 1
            if prev_node != None:
                prev_node_id = prev_node.id
            else:
                prev_node_id = -1

            #node.C.view(True, "Selected Node")
            #node.C.view_close()

            # Check if configuration is feasible
            f = False
            if type==0 and node.id != prev_node_id:
                if self.verbose > 0:
                    print("Checking if configuration is feasible")
                f, _ = self.run_rrt(node.C, node.C.frame(self.obj).getPosition()[0:2], node.FS, self.verbose, N=1)
                if f:
                    tac_coll = time.time()
                    print("SeGMan Time: ", tac_coll - tic_base)
                    for i in range(len(node.FS)- 10):
                        fs = node.FS[i]
                        self.FS.append(fs)  
                    return True, self.FS
                
            elif type==1 and node.id != prev_node_id:
                C2 = self.make_agent(node.C, self.obj)
                f, path = self.run_rrt(C2, self.goal, [], self.verbose, N=1, step_size=0.05)

                if f:
                    tac_coll = time.time()
                    print("SeGMan Time: ", tac_coll - tic_base)
                    for i in range(len(node.FS)):
                        fs = node.FS[i]
                        self.FS.append(fs) 

                    #self.display_solution(self.FS)
                    return True, path     
            
            prev_node = node

            # Check which objects are reachable in the pair
            any_reach = False
            for o in node.pair:
                is_reach, reach_P = self.is_reachable(node, o)
                if not is_reach:
                    #node.reachable_objs.remove(o)   
                    continue

                any_reach = True
                node.moved_obj = o

                if self.verbose > 0:
                    print(f"Object {o} is REACHABLE")

                # For the reachable object, generate subgoals
                # p1 -> radius: 1
                # a3-92 -> radius: 1
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
            #closest_idx = np.argmin(distances)
            #closest_dist = distances[closest_idx]
            #directed_graph.add_edge(path_names[i], path_names[closest_idx])
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

    def weight_collision_pairs_v2(self, cluster_dict:dict): 

        # Calculate cluster sizes
        cluster_sizes = {
            core_pair: sum(len(name) for name in cluster)
            for core_pair, cluster in cluster_dict.items()
        }
        max_cluster_size = max(cluster_sizes.values()) if cluster_sizes else 1

        # Group paths by cluster and compute scores
        weighted_obstacle_pairs = []
        for core_pair, cluster in cluster_dict.items():
            cluster_size = cluster_sizes[core_pair]
            normalized_cluster_size = cluster_size / max_cluster_size  # Normalize by max size

            # Compute scores for each pair in the cluster
            for name in cluster:
                pair_size = len(name)
                pair_score = 1 / pair_size  # Inverse of name length to favor shorter names
                core_score = 2 if core_pair == name else 1  # Double weight if it's the core pair
                final_score = core_score * pair_score / normalized_cluster_size

                # Store as Pair object
                weighted_obstacle_pairs.append(Pair(objects=name, weight=final_score))

                if self.verbose > 0:
                    print(f"Pair: {name}, Core Pair: {core_pair}, Cluster Size: {cluster_size}, Normalized Size: {normalized_cluster_size:.4f}, "
                        f"Pair Size: {pair_size},  "
                        f"Pair Score: {pair_score}, Score: {final_score:.4f}")

        # Sort by weight in descending order
        weighted_obstacle_pairs = sorted(weighted_obstacle_pairs, key=lambda pair: pair.weight, reverse=True)

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
            if isFirst:
                #self.verbose = 2
                self.node_score(node, obs_path, isFirst)
                #self.verbose = 1
            elif prev_node != None and node.pair == prev_node.pair:
                self.node_score(node)
            
            #print("TESTED NODE: ", node)
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

    def node_score(self, node:Node, obs_path:list=[], isFirst:bool=False):
        c0 = 5
        c1 = 5e-3
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

        if isFirst:
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
            if isFirst:
                #if self.obj_init_scene_score.get(o) == None:
                scene_score, pts = self.scene_score(node.C_hm, o + "_cam_g", self.verbose)
                node.pts = pts
                #self.obj_init_scene_score[o] = scene_score
                self.obj_pts[tuple(node.pair)] = pts
                #else:
                #    scene_score = self.obj_init_scene_score[o]
                #    node.pts = self.obj_pts[o]
                    
                node.init_scene_scores[o] = scene_score
                node.prev_scene_scores[o] = scene_score
                global_scene_score += scene_score
                temporal_scene_score += 0
            else:
                # New node
                if node.total_score == float('-inf'):
                    
                    scene_score, pts = self.scene_score(node.C_hm, o + "_cam_g", 2)
                    node.pts = pts
                    parent_visit = node.parent.visit
                    global_scene_score += scene_score #- node.init_scene_scores[o]
                    temporal_scene_score += scene_score - node.prev_scene_scores[o]
                    node.prev_scene_scores[o] = scene_score
        
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

    def make_agent(self, C:ry.Config, obj:str):
        C2 = ry.Config()
        C2.addConfigurationCopy(C)

        frame_names = C2.getFrameNames()
        for fn in frame_names:
            if "ego" in fn:
                C2.delFrame(fn)
        
        obj_f = C2.getFrame(obj)
        obj_J = C2.getFrame(obj+"Joint")

        pos = obj_f.getPosition()        
        obj_f.setJoint(ry.JT.transXY, limits=[-4, 4, -4, 4])
        
        obj_J.setPosition([0, 0, 0.2])
        C2.setJointState(pos[0:2])

        return C2
   
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def generate_subnode(self, node:Node, o:str, N:list, P:list=[], sample_count:int = 30, radius:int=0.6, pair:list=[]):
        
        if self.verbose > 0:
            print("Generate subgoals")

        Ct = ry.Config()
        Ct.addConfigurationCopy(node.C)

        obj_pos = node.C.frame(o).getPosition()
        feas_count = 0
        err_count = 0
        err_lim = 5
        min_dist = 0.3
        
        #self.verbose = 2

        if len(self.moved_pos) == 0:
            for n in N:
                if n.pair == node.pair:
                    self.moved_pos.append(n.C.frame(o).getPosition())

        fs_base = copy.deepcopy(node.FS)
        if len(P) > 0:
            for p in P:
                fs_base.append(p)
                Ct.setFrameState(p)

        # Filtering condition
        pts = copy.deepcopy(node.pts)

        pts = pts[::8]
        pts = pts[::1]
        random.shuffle(pts)

        generated_nodes = []

        for i in range(len(pts)):
            Ct = ry.Config()
            Ct.addConfigurationCopy(node.C)

            #angle = random.uniform(0, 2 * math.pi)  
            #r = random.uniform(0.1, radius)  
            if len(node.pts) == 0:
                print("NOOO POINTS")
                return 
            
            obj_pos_n = pts[i]#random.choice(selected_points)
            
            if self.verbose > 0:
                print(f"Trying for {i} / {len(pts)}, Error: {err_count}", f"Subnode: {feas_count} / {sample_count}" )

            obj_mov_pos = [obj_pos_n[0], obj_pos_n[1], obj_pos[2]]
            
            Ct.addFrame("cur_pos", "world", "shape: marker, size: [0.1]").setPosition(obj_mov_pos)    
            komo = ry.KOMO(Ct, phases=2, slicesPerPhase=10, kOrder=2, enableCollisions=True)                            # Initialize LGP
            komo.addControlObjective([], 1, 1e-1)
            komo.addControlObjective([], 2, 1e-1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)                                                                                        # Randomize the initial configuration
            komo.addObjective([0,1], ry.FS.distance, [self.agent, o], ry.OT.eq, scale=1e2, target=0)                                     # Pick
            komo.addModeSwitch([1,2], ry.SY.stable, [self.agent, o], True)   
            komo.addObjective([1,2] , ry.FS.positionDiff, [o, "cur_pos"], ry.OT.sos, scale=1e0)  # Place constraints 

            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
            Ct.delFrame("cur_pos")

            if self.verbose > 1:
                komo.view_play(True, f"Subgoal Generation Solution: {ret.eq < 1}")
            
            if ret.feasible:
                path_frames=komo.getPathFrames()

                fs = copy.deepcopy(fs_base)
                for p in path_frames:
                    fs.append(p)
                    Ct.setFrameState(p)

                self.moved_pos.append(Ct.frame(o).getPosition())

                new_node = Node(C=Ct, pair=node.pair, C_hm=node.C_hm, parent=node, layer=node.layer+1, FS=fs, init_scene_scores=node.init_scene_scores, prev_scene_scores=node.prev_scene_scores, moved_obj=o)

                multiplier = 1
                Ct2 = ry.Config()
                Ct2.addConfigurationCopy(Ct)
                Ct2.frame(o).unLink()
                
                for ob in node.pair:
                    if ob != o:
                        #print("Checking for ", ob)
                        Ct2.addFrame("cur_pos2", "world", "shape: marker, size: [0.1]").setPosition(Ct2.frame(ob).getPosition())
                        komo2 = ry.KOMO(Ct2, phases=2, slicesPerPhase=20, kOrder=2, enableCollisions=True)                           
                        komo2.addControlObjective([], 1, 1e-1)
                        komo2.addControlObjective([], 2, 1e-1)
                        komo2.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)                                                                                        
                        komo2.addObjective([0,1], ry.FS.distance, [self.agent, ob], ry.OT.eq, scale=1e2, target=0)                                     
                        komo2.addModeSwitch([1,2], ry.SY.stable, [self.agent, ob], True)   
                        komo2.addObjective([1,2] , ry.FS.positionDiff, [ob, "cur_pos2"], ry.OT.sos, scale=1e0)  
                        ret2 = ry.NLP_Solver(komo2.nlp(), verbose=0).solve() 
                        if ret2.eq < 1:                              
                            multiplier *= 2
                        Ct2.delFrame("cur_pos2")

                        #komo2.view_play(True, f"{ret2.eq}")
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
        
        #Ct.frame(o).setContact(0)
        #is_reachable, _ =self.run_rrt(Ct, node.C.frame(o).getPosition()[0:2], [], 2, N=1)
        is_reachable, _ = self.find_pick_path(node.C, self.agent, o, P, self.verbose, K=5, N=1)  
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
    def find_collision_pairs(self, type:int, obs_pair:list):
        if self.verbose > 0:
            print("Finding collision pairs")
        
        
        pair_path = {}
        obj_goal = self.C.frame(self.obj).getPosition()[0:2]
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
                pair_path[tuple(op)] = np.round(path, 2)

            if self.verbose > 1:
                print(f"Is {op} blocking path: {f}")



        if self.verbose > 0:
            print(f"The blocking obstacle pairs: {pair_path.keys()}")
                
        return pair_path

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
    def find_collision_pairs_v2(self, N:int=5):
        tic = time.time()

        if self.verbose > 0:
            print("Finding collision pairs")
        
        goal = self.C.frame(self.obj).getPosition()[0:2]
        pair_clusters = {}

        n = 0
        for op in self.obstacle_pairs:
            best_match = self.find_best_match(op, pair_clusters)

            if best_match is not None:
                pair_clusters[best_match].append(op)

            else:
                if n >= N:
                    continue

                Ct = ry.Config()
                Ct.addConfigurationCopy(self.C)

                if self.verbose > 0:
                    print(f"Trying: {op}")
                
                for o in op:
                    Ct.frame(o).setContact(0)
                
                Ct.frame(self.obj).setContact(0)

                f, _ = self.run_rrt(Ct, goal, [], self.verbose, N=1)

                if f:
                    n+=1
                    pair_clusters[tuple(op)] = [op]

                if self.verbose > 1:
                    print(f"Is {op} blocking path: {f}")


        
        print("Pair Clusters: ", pair_clusters)
        tac = time.time()
        print("Collision Pair Time: " , tac-tic)
        return pair_clusters
    
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
    
    def find_pick_path(self, C:ry.Config, agent:str, obj:str, FS:list, verbose: int, wp_f:list=[], K:int=5, N:int=5):

        if verbose > 0:
            print(f"Running Pick Path")

        for k in range(0, K):
            Ct = ry.Config()
            Ct.addConfigurationCopy(C)
            if verbose > 1:
                print(f"Trying Pick KOMO for {k}")
            
            S = ry.Skeleton()
            S.enableAccumulatedCollisions(True)
            if len(wp_f) == 0:
                #Ct.view(True, "Pick Path")
                #Ct.view_close()
                S.addEntry([0, 1], ry.SY.touch, [agent, obj])                                  # Pick
                komo = S.getKomo_path(Ct, 1, 1e-1, 1e-1, 1e-1, 1e2)
                if k != 0: 
                    komo.initRandom()  
            else:
                Ct.addFrame("subgoal", "world", "shape: marker, size: [0.1]").setPosition([*wp_f, 0.2])
                S.addEntry([0.1, -1], ry.SY.touch, [agent, obj])
                S.addEntry([0.2, -1], ry.SY.stable, [agent, obj])
                S.addEntry([0.3, 0.4], ry.SY.positionEq, ["subgoal", obj])
                komo = S.getKomo_path(Ct, 30, 1e1, 1e-1, 1e-1, 1e2)
                if k != 0: 
                    komo.initRandom()  
            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 

            if len(wp_f) > 0:
                Ct.delFrame("subgoal")

            if verbose > 1:
                komo.view_play(True, f"Pick Komo Solution: {ret.feasible}")
                komo.view_close()
                
            if ret.feasible:
                fr, P = self.run_rrt(C, komo.getPath()[-1], FS, verbose, N)
                if fr:
                    return True, P
        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def find_pick_komo(self, C:ry.Config, agent:str, obj:str, FS:list, verbose: int, K:int=20):
        Ct = ry.Config()
        Ct.addConfigurationCopy(C)
        if verbose > 1:
            print(f"Running Pick KOMO")
        for k in range(0, K):
            # Find a feasible touch configuration
            if verbose > 1:
                print(f"Trying Pick KOMO for {k}")
            komo = ry.KOMO(Ct, phases=1, slicesPerPhase=30, kOrder=2, enableCollisions=True)                            # Initialize LGP
            komo.addControlObjective([], 1, 1e-1)
            komo.addControlObjective([], 2, 1e-1)
            komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)                                                                                        # Randomize the initial configuration
            komo.addObjective([0,1], ry.FS.distance, [agent, obj], ry.OT.eq, scale=1e2, target=0)                                     # Pick
            if k != 0: 
                komo.initRandom()  
            ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 

            if verbose > 1:
                komo.view_play(True, f"Pick Komo Solution: {ret.feasible}")
                komo.view_close()
            if ret.feasible:
                for pf in komo.getPathFrames():
                    FS.append(pf)
                return True, komo.getPath()[-1]    
        return False, None

# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def path_check(self, C:ry.Config, agent, goal):

        Ct = ry.Config()
        Ct.addConfigurationCopy(C)

        komo = ry.KOMO(Ct, phases=1, slicesPerPhase=20, kOrder=2, enableCollisions=True)                            # Initialize LGP
        
        komo.addControlObjective([], 1, 1e-1)
        komo.addControlObjective([], 2, 1e-1)
        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e3)                                                                                        # Randomize the initial configuration
        komo.addObjective([1,2] , ry.FS.positionDiff, [agent, goal], ry.OT.eq, scale=1e1)  # Place constraints 
        ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 

        if self.verbose > 1:
            komo.view_play(True, f"Path End Check: {ret.feasible}")
            komo.view_close()

        if ret.feasible: 
            return True, komo.getPath()
        
        return False, None
    
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------- #

    def run_rrt(self, C:ry.Config, goal:list, FS:list, verbose: int, N:int=20, step_size:float=0.05):
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
                if verbose > 1:
                    Ct.view(True, "RRT Solution")
                for p in path:
                    Ct.setJointState(p)
                    FS.append(Ct.getFrameState())
                    if verbose > 1:
                        Ct.view(False, "RRT Solution")
                        time.sleep(0.01)
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
                    print(f"Trying Move KOMO for {k+1} time")

                komo = ry.KOMO(Ct, phases=2, slicesPerPhase=10, kOrder=2, enableCollisions=True)                            # Initialize LGP
                komo.addControlObjective([], 1, 1e-1)
                komo.addControlObjective([], 2, 1e-1)
                komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, scale=1e2)                                                                                        # Randomize the initial configuration
                komo.addObjective([0,1], ry.FS.distance, [agent, obj], ry.OT.eq, scale=1e2, target=0)                                     # Pick
                komo.addModeSwitch([1,2], ry.SY.stable, [agent, obj], True)   
                komo.addObjective([1,2] , ry.FS.positionDiff, [obj, "subgoal"], ry.OT.sos, scale=1e0)  # Place constraints 
                if k != 0:
                    komo.initRandom()
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
                elif k == K-1:
                    return False, Ct, wp
                
            if pi == len(P)-1:
                return True, Ct, None
            
        return False, None, None

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
        #img, depth = SeGMan.get_image(C, cam_frame, self.verbose)
        scene_score = 0
        for i, r in enumerate(img):
            for j, rgb in enumerate(r):
                #if rgb[0] > 200 and rgb[1] < 200 and rgb[2] < 200:
                #    scene_score += 0
                #elif rgb[1] > 200 and rgb[0] > 200 and rgb[2] < 200:
                #    scene_score += 0 
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
        if(verbose>2):
            C2 = ry.Config()
            C2.addFrame("world")
            C.getFrame("world").setPointCloud(pts, [0,0,0])
            
            C.view(True,"1")
            C2.getFrame("world").setPointCloud(pts, [0,0,0])
            C2.view(True,"2")

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

    @staticmethod
    def get_image(C:ry.Config, cam_frame:str, verbose:int):
        camera_view = ry.CameraView(C)
        cam = C.getFrame(cam_frame)
        camera_view.setCamera(cam)
        img, depth = camera_view.computeImageAndDepth(C)
        img = np.asarray(img)

        if(verbose > 1):
            fig = plt.figure()
            fig.suptitle(f"Cam Frame: {cam_frame}", fontsize=16)
            plt.imshow(img)
            plt.show()

        return img, depth
    
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