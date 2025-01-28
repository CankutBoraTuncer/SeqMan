import robotic as ry
from utils import *
from openai import OpenAI
import os
import numpy as np

if __name__ == "__main__":

    prompt = """You are provided with a 2d puzzle setting. Here is the supplementary information about the puzzle:
- The yellow disk is an agent that can move in XY directions with 2-dof, positioned at [-1.3, -1.3]
- The agent cannot rotate
- The blue square is the target object positioned at [1.6, -0.5]
- The red square is the goal position at [-1.5, 1.5]
- The pink and green rectangles are movable obstacles
- The brown thick lines are walls
- The agent can attach to the target object or any movable obstacle from any angle and move with it
- Neither the agent nor the target object is allowed to collide with the walls
- The task is to move the target object to the goal position using agent's pick-place movements
- The white dot grid or line grid uses 0.2 unit distance between each line and is given for visual reference for your measurements
- The left side is -x direction, the right side is +x direction
- The bottom side is -y direction, the top side is +y direction
- The camera is positioned at [0, 0] i.e. the center of the grid

Goal Position: [-1.5, 1.5]
Target Object Position: [1.6, -0.5]

Your task is to identify the necessary path that the agent must take from its starting position towards the target object and place it to the goal position. If this path is obstructed by any movable obstacle, identify the obstacles that need to be moved and suggest an appropriate position to move them to. Every action must be given as object-position pairs formatted as "(<object-name>, [x, y])". Object names are color names \{blue, pink, green\} and position are coordinates with floating point x and y. The agent will pick the object given and move it to the given position. If there are multiple manipulations that need to be done, give them in the correct order of placements with the first pair moved first and last pair moved last.

----

INSTRUCTIONS:
- analyze the given image
- use reasoning and your understanding of movement paths and collisions to identify the path that the agent must take
- no object can collide with the walls
- goal positions don't need to be aligned to the grid
- explain your reasoning
- Finally, give your answer as an array of ordered pairs after the keyword ANSWER"""

    CONFIG_NAME = "p1-two-blocks"

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
                            "text": prompt,
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{base64_image}"},
                        },
                    ],
                }
            ],
            max_completion_tokens=1000
        )

        print(f"{CONFIG_NAME} - {i}")
        print(response.choices[0].message.content)
        print("\n\n")
    