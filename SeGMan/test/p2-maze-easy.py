import robotic as ry
import sys
sys.path.append('..')
from src.SeGMan import SeGMan
import time

if __name__ == "__main__":
    C = ry.Config()
    C.addFile("../src/config/p2-maze-easy/p2-maze-easy.g")

    EGO_NAME = "ego"
    OBJ_NAME = "obj"
    GOAL     = C.getFrame("goal").getPosition()[0:2]
    OBS_LIST = []
    C.view(True)
    C.view_close()

    segman = SeGMan(C, None, EGO_NAME, OBJ_NAME, GOAL, OBS_LIST, 0)
    if segman.run():
        segman.display_solution(pause = 0.02)






