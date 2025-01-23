import robotic as ry
from utils import *
from Node import Node
import base64
from openai import OpenAI
import os
import numpy as np

if __name__ == "__main__":
    C0 = ry.Config()
    C0.addFile("config/p6-wall.g")
    C0.addFile("config/camera-top.g")
    # C0.view(True)
    # C0.view_close()

    for x in np.linspace(-2.0, 2.0, 21):
        for y in np.linspace(-2.0, 2.0, 21):
            C0.addFrame(f"grid_{x}_{y}")\
                .setPosition([x, y, 0.1])\
                .setShape(ry.ST.sphere, size=[.05])\
                .setColor([1, 1, 1])\
                .setContact(0)
            
    C0.view()
    C0.view_setCamera(C0.getFrame("camera-top"))
    C0.view(True)
    C0.view_close()

    # image = overlay_grid_over_image(C0, 20, camera_name="camera-top", camera_path="config/camera-top.g")
    # plt.imsave(f"tmp/grid_{int(np.random.uniform(0, 10000))}.png", image)

    del C0
    # client = get_client(os.environ["OPENAI_API_KEY"])

    # # Path to your image
    # image_path = "tmp/grid.png"

    # # Getting the base64 string
    # base64_image = encode_image(image_path)

    # response = client.chat.completions.create(
    #     model="gpt-4o-mini",
    #     messages=[
    #         {
    #             "role": "user",
    #             "content": [
    #                 {
    #                     "type": "text",
    #                     "text": "What is in this image?",
    #                 },
    #                 {
    #                     "type": "image_url",
    #                     "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
    #                 },
    #             ],
    #         }
    #     ],
    # )

    # print(response.choices[0])
    