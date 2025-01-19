import robotic as ry

if __name__ == "__main__":
    C = ry.Config()
    #C.watchFile("../src/config/p4-four-blocks/p4-four-blocks.g")
    C.addFile("../src/config/p4-four-blocks/p4-four-blocks.g")

    C.view(True, "pre")

    C.frame("obj1").setJoint(ry.JT.transXY, limits=[-4, 4, -4, 4])
    C.view(True, "post")








