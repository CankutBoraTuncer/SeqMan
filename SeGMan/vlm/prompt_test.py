import robotic as ry
from utils import *
from pydantic import BaseModel
import os
import json
import sys


class ObstaclePositionPair(BaseModel):
    reasoning: str
    obstacle_id: str
    obstacle_move_position: list[float]


class VLMResponse(BaseModel):
    task: str
    puzzle_description: str
    path_explanation: str
    obstructing_obstacles_ids: list[str]
    obstacle_position_pairs: list[ObstaclePositionPair]


TASK_A2O = "agent to object"
TASK_O2G = "object to goal"

# This function builds the user prompt from the given dictionary. The dictonary is 
# generated from the generate_dict function in utils. And contains the task and
# positional information about the agent, object, goal, and obstacles.
def build_prompt(prompt_dict: dict) -> str:
    puzzle_info_str = json.dumps(prompt_dict)
    return puzzle_info_str


SYSTEM_PROMPT = """Your task is to analyze a 2D pick-and-place puzzle environment, given in a top-down view with the camera positioned on top of [0.0, 0.0] (center), and plan a sequence of actions for an agent to move an object to a goal position without causing collisions. 

The coordinate system is as follows: +x is to the right, +y is upwards, and the origin [0.0, 0.0] is at the center.

Identify and distinguish between the agent, the target object, the goal position, movable obstacles, and any obstacles obstructing the path. Propose a step-by-step plan for the agent to accomplish the task, ensuring each move respects the environmental constraints. The scene is surrounded by walls with endpoints at [(-2.0, -2.0), (-2.0, 2.0), (2.0, -2.0), (2.0, 2.0)] which cannot be crossed by any object within the environment. There may also be additional walls with dark brown color.

# Steps

1. **Scene Analysis**:  
   - Identify all key elements: agent, target object, goal position, and obstacles.
   - Determine the positions of these elements using the provided white grid with 0.2 ticks and coordinate system.

2. **Path Planning**:  
   - Assess whether the target can be moved directly to the goal without obstacle manipulation.
   - If obstacles block the path, calculate the sequence of moves to clear them.

3. **Reasoning and Subgoals**:  
   - For each obstructing obstacle, reason why it must be moved.
   - Establish a new position for each obstacle that does not impede any necessary paths.

4. **Action Proposal**:  
   - Formulate obstacle-position pairs with clear reasoning.

# Output Format

Your result must be in JSON format:

```json
{
    "task": "The given task",
    "puzzle_description": "A brief description of the current puzzle setup.",
    "path_explanation": "A step-by-step explanation of how the path is cleared or why no obstacles need movement.",
    "obstructing_obstacles_ids": ["ID1", "ID2", ...],
    "obstacle_position_pairs": [
        {
            "reasoning": "Reason for moving the obstacle.",
            "obstacle_id": "Unique identifier of the obstacle.",
            "obstacle_move_position": [new_x, new_y]
        },
        ...
    ]
}
```

# Examples

**Example Input**:  

"..."

**Example Output**:

```json
{
    "task": "agent to object",
    "puzzle_description": "The yellow agent is at the center of the puzzle. The target object is at the upper right corner surrounded by walls and obstacles. The goal position is at the lower half of the puzzle and is free from obstacles. There are two obstacles near the target object, one on the left and one on the right.",
    "path_explanation": "The target object is blocked from left and right by obstacles, and top and bottom by walls. The agent must relocate one of the obstacles to reach the target object and then move the target object to the goal position. obstacle_1 can be moved out of the way easily to clear the path.",
    "obstructing_obstacles_ids": ["obstacle_1"],
    "obstacle_position_pairs": [
        {
            "reasoning": "The obstacle blocks the left side of the target object.",
            "obstacle_id": "obstacle_1",
            "obstacle_move_position": [-1.1, -1.63]
        }
    ]
}
```

# Notes

- Ensure reasoning is clear for each move.
- Leave "obstructing_obstacles_ids" and "obstacle_position_pairs" empty if no obstacles require moving.
- Decimal values can be used for precise positioning within the coordinate system.
- The anchor of each object is its center, not its corner."""


if __name__ == "__main__":

    CONFIG_NAME = "p1-two-blocks"
    if len(sys.argv) > 1:
        CONFIG_NAME = sys.argv[1]

    overlay_parameters = {
        "image" : ["overlay_matrix.png", "overlay_grid.png"],
        "opacity" : [0.5, 0.5, 1, 1],
    }
        
    client = get_client(os.environ["OPENAI_API_KEY"])

    for i in range(4):
        image_path = f"tmp/{CONFIG_NAME}_{i}.png"

        C0 = ry.Config()
        C0.addFile(f"../src/config/{CONFIG_NAME}/{CONFIG_NAME}.g")

        prompt = build_prompt(generate_dict(
            TASK_A2O,
            C0,
            agent_name="ego",
            obj_name="obj",
            goal_name="goal"
        ))
        if not os.path.exists(image_path):
            overlay_image_over_config(
                C0,
                overlay_parameters["image"][i%2],
                image_path,
                opacity=overlay_parameters["opacity"][i],
                camera_name="camera-top",
                camera_path="../src/config/camera-top.g"
            )

        else:
            print(f"Image already exists: {image_path}")
            

        # Getting the base64 string
        base64_image = encode_image(image_path)

        response = client.beta.chat.completions.parse(
            model="gpt-4o",
            messages=[
                {
                    "role": "system",
                    "content": [
                        {
                            "type": "text",
                            "text": SYSTEM_PROMPT,
                        }
                    ]
                },
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
            response_format=VLMResponse,
            max_completion_tokens=2000
        )

        # vlm_response is an instance of VLMResponse
        vlm_response = response.choices[0].message.parsed

        print(f"{CONFIG_NAME} - {i}")
        print(prompt)
        print()
        print(json.dumps(vlm_response.model_dump(), indent=2))
        print("\n\n")

        if vlm_response.obstructing_obstacles_ids:
            obstacle_ids = vlm_response.obstructing_obstacles_ids
            opp_list = vlm_response.obstacle_position_pairs
            
            for opp in opp_list:
                id = opp.obstacle_id
                new_position = opp.obstacle_move_position
                C0.addFrame("new_"+id)\
                    .setShape(ry.ST.cylinder, [2.0, 0.05])\
                    .setColor(C0.getFrame(id).getAttributes()["color"])\
                    .setPosition([new_position[0], new_position[1], 0.1])
            
            C0.view(True)
    
        del C0