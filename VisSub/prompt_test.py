import robotic as ry
from utils import *
from Node import Node
from openai import OpenAI
import os
import numpy as np

if __name__ == "__main__":

    # C0 = ry.Config()
    # C0.addFile("config/p6-wall.g")
    # overlay_image_over_config(
    #     C0,
    #     "overlay_matrix.png",
    #     "tmp/test003.png",
    #     opacity=1
    # )
    # del C0
        

    client = get_client(os.environ["OPENAI_API_KEY"])

    for i in range(1, 5):
        # Path to your image
        image_path = f"tmp/test00{i}.png"

        # Getting the base64 string
        base64_image = encode_image(image_path)

        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "Describe what you see in the image.",
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                        },
                    ],
                }
            ],
            max_completion_tokens=300
        )

        print(response.choices[0].message.content)
    