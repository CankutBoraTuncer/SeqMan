import robotic as ry
from utils import *
from openai import OpenAI
import os
import numpy as np

if __name__ == "__main__":

    CONFIG_NAME = "p9-cube-free"

    overlay_parameters = {
        "image" : ["overlay_matrix.png", "overlay_grid.png"],
        "opacity" : [0.5, 0.5, 1, 1],
    }
        
    client = get_client(os.environ["OPENAI_API_KEY"])

    for i in range(4):
        image_path = f"tmp/{CONFIG_NAME}_{i}.png"

        if not os.path.exists(image_path):
            C0 = ry.Config()
            C0.addFile(f"../src/config/{CONFIG_NAME}/{CONFIG_NAME}.g")
            overlay_image_over_config(
                C0,
                overlay_parameters["image"][i%2],
                image_path,
                opacity=overlay_parameters["opacity"][i],
                camera_name="camera-top",
                camera_path="../src/config/camera-top.g"
            )

            del C0
        else:
            print(f"Image already exists: {image_path}")

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
                            "image_url": {"url": f"data:image/png;base64,{base64_image}"},
                        },
                    ],
                }
            ],
            max_completion_tokens=300
        )

        print(f"{CONFIG_NAME} - {i}")
        print(response.choices[0].message.content)
        print("\n\n")
    