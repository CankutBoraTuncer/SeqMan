import robotic as ry
import sys
sys.path.append('..')
from src.SeGMan import SeGMan
import time

if __name__ == "__main__":
    C = ry.Config()
    C.addFile("../src/config/p3-maze/p3-maze.g")

    EGO_NAME = "ego"
    OBJ_NAME = "obj"
    GOAL     = C.getFrame("goal").getPosition()[0:2]
    OBS_LIST = []
    C.view(True)
    C.view_close()

    segman = SeGMan(C, None, EGO_NAME, OBJ_NAME, GOAL, OBS_LIST, 1)
    if segman.run():
        segman.display_solution(pause = 0.02)






