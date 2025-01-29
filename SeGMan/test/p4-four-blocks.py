import robotic as ry
import sys
sys.path.append('..')
from src.SeGMan import SeGMan
import time

if __name__ == "__main__":
    C = ry.Config()
    C.addFile("../src/config/p4-four-blocks/p4-four-blocks.g")
    C_hm = ry.Config()
    C_hm.addFile("../src/config/p4-four-blocks/p4-four-blocks-heatmap.g")

    EGO_NAME = "ego"
    OBJ_NAME = "obj"
    GOAL     = C.getFrame("goal").getPosition()[0:2]
    OBS_LIST = ["obj1", "obj2", "obj3", "obj4"]
    C.view(True)
    C.view_close()

    segman = SeGMan(C, C_hm, EGO_NAME, OBJ_NAME, GOAL, OBS_LIST, 1)
    tic = time.time()
    if segman.run():
        toc = time.time()
        print("Time elapsed: {} seconds".format(toc - tic))
        segman.display_solution(pause = 0.2)






