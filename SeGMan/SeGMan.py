import robotic as ry
import time
import os
from contextlib import contextmanager

class HMAP:

    def __init__(self, C:ry.Config, C2:ry.Config, qF:list, q_obs:list, target:str, obs_count:int, tool_list:list, agent_list:list, verbose:int=0):
        self.C = C
        self.C2 = C2
        self.qF = qF
        self.q_obs = q_obs
        self.target = target
        self.obs_count = obs_count
        self.tool_list = tool_list
        self.agent_list = agent_list
        self.verbose = verbose
        self.komo_path = []
        self.is_soln_found = False

    def run(self):
        rrt_try = 0
        rrt_path = []
        agent = self.agent_list[0]

        # Find path to goal for the object
        while rrt_try < 30:
            with self.suppress_stdout():  
                rrt = ry.PathFinder()                                          
                rrt.setProblem(self.C2, [self.C2.getJointState()], [self.qF])
                solution = rrt.solve()
            if solution.feasible:
                rrt_path = solution.x
                break
            rrt_try += 1
        
        if solution.feasible:

            if self.verbose > 1:
                self.C2.view(True, "RRT Path")
                for js in rrt_path:
                    self.C2.setJointState(js)
                    self.C2.view(False, f"RRT Path | Try: {rrt_try}")
                    time.sleep(0.0005)
            
            is_pick_feas = False
            for pick_try in range(30): 
                # Find the path from agent to the object
                S = ry.Skeleton()
                S.enableAccumulatedCollisions(True)
                S.addEntry([0, 1], ry.SY.touch, [agent, self.target])
                
                komo = S.getKomo_path(self.C, 1, 1e-1, 1e1, 1e-1, 1e2) 
                komo.initRandom()  
                ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve()                                                 

                if self.verbose > 0:
                    komo.view_play(True, f"Pick Komo | Try: {pick_try} | Feasible: {ret.feasible}")
                    komo.view_close()

                if ret.feasible :
                    rrtC = ry.Config()
                    rrtC.addConfigurationCopy(self.C)
                    with self.suppress_stdout(): 
                        rrt2 = ry.PathFinder()                    
                        rrt2.setProblem(rrtC, rrtC.getJointState(), komo.getPath()[-1])
                        solution2 = rrt2.solve()
                    if solution2.feasible:
                        is_pick_feas = True
                        for js in solution2.x:
                            self.C.setJointState(js)
                            if self.verbose > 0:
                                self.C.view(False, f"Pick RRT | Try: {pick_try}")
                                self.komo_path.append(self.C.getFrameState())
                                time.sleep(0.005)
                        break
                    
            wp = 0
            fail_count = 0

            while wp < len(rrt_path) and is_pick_feas and wp >= 0 and fail_count < 5:
                if self.verbose > 0:
                    print(f"Waypoint {wp+1}/{len(rrt_path)}")

                

                sg = self.C.addFrame("subgoal", "world", "shape: marker, size: [0.1]")
                sg.setPosition([*rrt_path[wp], 0.2])
                S = ry.Skeleton()
                S.enableAccumulatedCollisions(True)

                S.addEntry([0.1, -1], ry.SY.touch, [agent, self.target])
                S.addEntry([0.2, -1], ry.SY.stable, [agent, self.target])
                S.addEntry([0.3, 0.4], ry.SY.positionEq, ["subgoal", self.target])
                komo = S.getKomo_path(self.C, 10, 1e-3, 1e-3, 1e-5, 1e2)
                ret = ry.NLP_Solver(komo.nlp(), verbose=0).solve() 
                
                self.C.delFrame("subgoal")

                if ret.eq < 1:
                    wp += 1
                    fail_count = 0
                    pf = komo.getPathFrames()
                    self.C.setFrameState(pf[-1])
                    self.komo_path.append(pf)
                else:
                    fail_count += 1
                    wp = wp - 10
                    self.C.setFrameState(self.komo_path[wp])
                    komo.report(True, True, True)
                    komo.view_play(True, f"Failed, {wp}")
                
            if wp == len(rrt_path):
                print("Solution found")
                self.is_soln_found = True
            else:
                print("No solution found")

    def display_solution(self):
        if self.is_soln_found:
            self.C.view(True, "Solution")
            for fs in self.komo_path:
                self.C.setFrameState(fs)
                self.C.view(False, "Solution")
                time.sleep(0.005)
                
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