import robotic as ry
import numpy as np
import os
from contextlib import contextmanager
import base64
from openai import OpenAI
from PIL import Image, ImageChops
import json

@contextmanager
def suppress_stdout():
    """Suppress output from C++ std::cout and std::cerr."""
    # Open /dev/null
    devnull = os.open(os.devnull, os.O_WRONLY)
    # Save the original stdout and stderr file descriptors
    original_stdout = os.dup(1)
    original_stderr = os.dup(2)
    
    try:
        # Redirect stdout and stderr to /dev/null
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        # Restore stdout and stderr to their original file descriptors
        os.dup2(original_stdout, 1)
        os.dup2(original_stderr, 2)
        # Close the duplicate file descriptors
        os.close(devnull)
        os.close(original_stdout)
        os.close(original_stderr)


def rgb_to_hex(rgb):
    return "#{:02x}{:02x}{:02x}".format(int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255))


# Function to read variables from .config
def generate_dict(task: str, config: ry.Config, agent_name: str = "ego", obj_name: str = "obj", goal_name: str = "goal") -> dict:
    res = {
        "task": task,
        "agent": {},
        "target object": {},
        "obstacles": [],
        "goal position": {}
    }
    frame_names = config.getFrameNames()
    for frame_name in frame_names:
        frame = config.getFrame(frame_name)
        attributes = frame.getAttributes()
        if frame_name == agent_name:
            res["agent"]["color"] = rgb_to_hex(attributes["color"])
            res["agent"]["position"] = frame.getPosition()[:2].tolist()
        elif frame_name == obj_name:
            res["target object"]["color"] = rgb_to_hex(attributes["color"])
            res["target object"]["position"] = frame.getPosition()[:2].tolist()
        elif frame_name == goal_name:
            res["goal position"] = frame.getPosition()[:2].tolist()
        elif "logical" in attributes and "object" in attributes["logical"]:
            res["obstacles"].append({
                "name": frame_name,
                "position": frame.getPosition()[:2].tolist()
            })
    return res
    

# Function to get the OpenAI client
def get_client(api_key: str) -> OpenAI:
    return OpenAI(api_key=api_key)


# Function to encode the image
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")
    

# Function to create a config to use for grid overlay image
def get_overlay_config_grid(
        low:float = -2.0,
        high:float = 2.0,
        count:int = 21,
        camera_path:str="config/camera-top.g") -> ry.Config:
    
    size = high - low
    config = ry.Config()
    config.addFile(camera_path)
    config.addFrame("base")\
        .setShape(ry.ST.box, size=[size*1.25, size*1.25, 0.05])\
        .setColor([0, 0, 0])\
        .setContact(0)
    
    for x in np.linspace(low, high, count):
        config.addFrame(f"grid_x_{x}")\
            .setPosition([x, 0, 0.1])\
            .setShape(ry.ST.box, size=[0.01, size, 0.05])\
            .setColor([1, 1, 1])\
            .setContact(0)
        
    for y in np.linspace(low, high, count):
        config.addFrame(f"grid_y_{y}")\
            .setPosition([0, y, 0.1])\
            .setShape(ry.ST.box, size=[size, 0.01, 0.05])\
            .setColor([1, 1, 1])\
            .setContact(0)

    return config


# Function to create a config to use for dot matrix overlay image
def get_overlay_config_matrix(
        low:float = -2.0,
        high:float = 2.0,
        count:int = 21,
        camera_path:str="config/camera-top.g") -> ry.Config:
    
    size = high - low
    config = ry.Config()
    config.addFile(camera_path)
    config.addFrame("base")\
        .setShape(ry.ST.box, size=[size*1.25, size*1.25, 0.05])\
        .setColor([0, 0, 0])\
        .setContact(0)
    
    for x in np.linspace(low, high, count):
        for y in np.linspace(low, high, count):
            config.addFrame(f"grid_{x}_{y}")\
                .setPosition([x, y, 0.1])\
                .setShape(ry.ST.sphere, size=[.03])\
                .setColor([1, 1, 1])\
                .setContact(0)
            
    return config


def overlay_image_over_config(
        C0:ry.Config,
        overlay_path:str,
        output_path:str,
        opacity:float=0.5,
        camera_name:str="camera-top",
        camera_path:str="config/camera-top.g") -> None:

    C0.addFile(camera_path)
    camera_view = ry.CameraView(C0)
    cam = C0.getFrame(camera_name)
    camera_view.setCamera(cam)
    img, depth = camera_view.computeImageAndDepth(C0)
    img = np.asarray(img)

    background = Image.fromarray(img)
    overlay = Image.open(overlay_path)

    background = background.convert("RGBA")
    overlay = overlay.convert("RGBA")

    opacity = np.clip(opacity, 0, 1)

    overlay = Image.blend(Image.new("RGBA", overlay.size, (0, 0, 0, 0)), overlay, opacity)

    result = ImageChops.screen(background, overlay)

    result.save(output_path, "PNG")
