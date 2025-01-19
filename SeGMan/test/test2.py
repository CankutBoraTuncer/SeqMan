import robotic as ry
import sys
sys.path.append('..')
from src.SeGMan import SeGMan

if __name__ == "__main__":
    C = ry.Config()
    C.addFile("../src/config/p4-four-blocks/p4-four-blocks-heatmap.g")
    SeGMan.scene_score(C, "obj1_cam_g", 2)    







