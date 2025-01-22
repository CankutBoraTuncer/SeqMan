import robotic as ry
from utils import *
from Node import Node
import base64
from openai import OpenAI
import os

if __name__ == "__main__":
    client = get_client(os.environ["OPENAI_API"])

    # Path to your image
    image_path = "tmp/grid.png"

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
                        "text": "What is in this image?",
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                    },
                ],
            }
        ],
    )

    print(response.choices[0])
    