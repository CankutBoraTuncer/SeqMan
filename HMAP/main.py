import robotic as ry
from HMAP import HMAP
import time
ry.params_add({
    "rrt/stepsize": 0.005,
    "rrt/verbose": -1,
})

if __name__ == "__main__":
    task = "../config/p5-wall-easy.g"
    task_actuated = "../config/p5-wall-easy-actuated.g"
    C = ry.Config()
    C.addFile(task)
    C2 = ry.Config()
    C2.addFile(task_actuated)

    EGO_NAME = "ego"
    OBJ_NAME = "obj"
    C2.setJointState(C.getFrame(OBJ_NAME).getPosition()[0:2])
    C.view(True)
    C.view_close()

    hmap = HMAP(C, C2, C2.frame("goal").getPosition()[0:2], [], OBJ_NAME, 0, [], [EGO_NAME], 0)

    start_time = time.time()
    hmap.run()
    end_time = time.time()
    execution_time = end_time - start_time
    print(f"Execution time: {execution_time:.6f} seconds")

    hmap.display_solution()


