import robotic as ry
import sys
sys.path.append('..')
from src.SeGMan import SeGMan

if __name__ == "__main__":
    C = ry.Config()
    C.addFile("../src/config/p3-maze/p3-maze.g")
    C2 = ry.Config()
    C2.addFile("../src/config/p3-maze/p3-maze-actuated.g")

    EGO_NAME = "ego"
    OBJ_NAME = "obj"
    GOAL     = C.getFrame("goal").getPosition()[0:2]
    C2.setJointState(C.getFrame(OBJ_NAME).getPosition()[0:2])
    C.view(True)
    C.view_close()

    segman = SeGMan(C, C2, EGO_NAME, OBJ_NAME, GOAL, [], 0)
    if segman.run():
        segman.display_solution()






