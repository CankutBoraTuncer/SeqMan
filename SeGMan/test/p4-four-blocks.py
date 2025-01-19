import robotic as ry
import sys
sys.path.append('..')
from src.SeGMan import SeGMan

if __name__ == "__main__":
    C = ry.Config()
    C.addFile("../src/config/p4-four-blocks/p4-four-blocks.g")
    C2 = ry.Config()
    C2.addFile("../src/config/p4-four-blocks/p4-four-blocks-actuated.g")
    C3 = ry.Config()
    C3.addFile("../src/config/p4-four-blocks/p4-four-blocks-heatmap.g")

    EGO_NAME = "ego"
    OBJ_NAME = "obj"
    GOAL     = C.getFrame("goal").getPosition()[0:2]
    OBS_LIST = ["obj1", "obj2", "obj3", "obj4"]
    C2.setJointState(C.getFrame(OBJ_NAME).getPosition()[0:2])
    C.view(True)
    C.view_close()

    segman = SeGMan(C, C2, EGO_NAME, OBJ_NAME, GOAL, OBS_LIST, 1)
    if segman.run():
        segman.display_solution()






