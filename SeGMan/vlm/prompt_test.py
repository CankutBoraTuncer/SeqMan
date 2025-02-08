import robotic as ry
from utils import *
from openai import OpenAI
from pydantic import BaseModel
import os
import numpy as np
import json


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

def build_prompt(task: str, agent_color, agent_pos, obj_color, obj_pos, obstacles: list, goal_pos):
    # task: {agent to object, object to goal}
    # grid & coordinate system
    # agent pos
    # object pos
    # which one is object
    # (obstacle identifier, obstacle pos)
    # goal pos

    prompt_dict = {
        "task": task,
        "agent": {
            "color": agent_color,
            "position": agent_pos,
        },
        "target object": {
            "color": obj_color,
            "object": obj_pos,
        },
        "obstacles": obstacles, # obstacle id & position
        "goal position": goal_pos
    }

    puzzle_info_str = json.dumps(prompt_dict)
    
    return puzzle_info_str


# SYSTEM_PROMPT = """You will be provided with a top-down view of a 2D pick-and-place puzzle. The scene is surrounded by walls with endpoints at [(-2.0, -2.0), (-2.0, 2.0), (2.0, -2.0), (2.0, 2.0)] which cannot be crossed by any object within the environment. Each puzzle contains an agent, a target object, a goal position, and movable obstacles. The agent can only move in XY directions with 2-dof, and cannot rotate. The agent can pick an obstacle or the target object and move them somewhere else. The agent or any object must not collide with anything else as a rule. The agent can pick an object an move it, and can repeat these pick-and-place movements multiple times. The puzzle is solved when the target object is above the goal position.

# The scene is overlaid with a grid with tick with 0.2 unit length to provide you a coordinate reference system. But nothing is constrained to the grid and positions can take decimal values.

# Your task is to analyze the scene, distinguish and identify the objects, reason a path (possibly multi-step) for the agent to carry out the given task, identify obstacles that obstruct the path and propose positions for the object in the correct order as subgoals. When you provide obstacle-position pairs; the agent will attempt to move to the obstacle, pick it and place it to the proposed position, and continue with the processing of the next pair until the list is processed completely. If the given task can be solved without any manipulation to the obstacles, then leave your response empty.

# The structure of your result must be a valid json in the form:
# {
#     task: str,
#     puzzle_description: str,
#     path_explanation: str,
#     obstructing_obstacles_ids: list[str],
#     obstacle_position_pairs: list[ObstaclePositionPair]
# }

# ObstaclePositionPair is of the form:
# {
#     reasoning: str,
#     obstacle_id: str,
#     obstacle_move_position: [float, float]
# }
# """

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
    "task": "object to goal.",
    "puzzle_description": "A straightforward layout with one notable obstacle requiring maneuver.",
    "path_explanation": "The target's direct path is blocked. Removing the obstacle at position X will clear the path.",
    "obstructing_obstacles_ids": ["obstacle_1"],
    "obstacle_position_pairs": [
        {
            "reasoning": "The obstacle is directly in the planned path of the target.",
            "obstacle_id": "obstacle_1",
            "obstacle_move_position": [0.0, 1.4]
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

    overlay_parameters = {
        "image" : ["overlay_matrix.png", "overlay_grid.png"],
        "opacity" : [0.5, 0.5, 1, 1],
    }
        
    client = get_client(os.environ["OPENAI_API_KEY"])

    for i in range(4):
        image_path = f"tmp/{CONFIG_NAME}_{i}.png"

        C0 = ry.Config()
        C0.addFile(f"../src/config/{CONFIG_NAME}/{CONFIG_NAME}.g")
        agent_color = (0.96875, 0.7421875, 0.30859375)
        agent_pos = C0.getFrame("ego").getPosition()[:2].tolist()
        obj_color = (0, 0, 1.0)
        obj_pos = C0.getFrame("obj").getPosition()[:2].tolist()
        obstacles = [("obj1", C0.getFrame("obj1").getPosition()[:2].tolist()),
                    ("obj2", C0.getFrame("obj2").getPosition()[:2].tolist())]
        goal_pos = C0.getFrame("goal").getPosition()[:2].tolist()

        prompt = build_prompt(
            task=TASK_A2O,
            agent_color=agent_color,
            agent_pos=agent_pos,
            obj_color=obj_color,
            obj_pos=obj_pos,
            obstacles=obstacles,
            goal_pos=goal_pos
            )
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

        print(f"{CONFIG_NAME} - {i}")
        print(prompt)
        print()
        print(response.choices[0].message.content)
        print("\n\n")

        vlm_response = response.choices[0].message.parsed
        if vlm_response.obstructing_obstacles_ids:
            obstacle_ids = vlm_response.obstructing_obstacles_ids
            opp_list = vlm_response.obstacle_position_pairs
            
            for opp in opp_list:
                id = opp.obstacle_id
                new_position = opp.obstacle_move_position
                C0.addFrame("new_"+id)\
                    .setShape(ry.ST.cylinder, [1.0, 0.1])\
                    .setColor([0, 0, 0])\
                    .setPosition([new_position[0], new_position[1], 0.1])
            
            C0.view(True)
    
        del C0