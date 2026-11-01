"""Minimal Python client: RPC, map, wheels, goto, state and camera streams.

pip install pyzmq numpy pillow; python client.py [--host localhost]
"""
import argparse
import io
import json
import math
import time

import numpy as np
import zmq
from PIL import Image
import asyncio

from motion.MovementStatus import MovementStatus
from motion.VacuumRobot import VacuumRobot
from helper.PixelMap import PixelMap
from helper.BoundingBox import BoundingBox
from helper.Goal import Goal
from helper.Path import Path
from helper.RecorderService import RecorderService
from PathFinder import PathFinder

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="localhost")
ap.add_argument("--port-base", type=int, default=5555)
ap.add_argument("--scene", default="01", help="Scene ID to load (e.g. '06')")
ap.add_argument("--seed", type=int, default=42, help="Random seed for start pose")
args = ap.parse_args()

# start of URL utility methods
def url(k):
    return f"tcp://{args.host}:{args.port_base + k}"


ctx = zmq.Context()
rpc = ctx.socket(zmq.REQ)
rpc.connect(url(0))
cmd = ctx.socket(zmq.PUSH)
cmd.connect(url(1))
state = ctx.socket(zmq.SUB)
state.connect(url(2))
state.setsockopt(zmq.SUBSCRIBE, b"")
camera = ctx.socket(zmq.SUB)
camera.setsockopt(zmq.RCVHWM, 2)
camera.connect(url(3))
camera.setsockopt(zmq.SUBSCRIBE, b"")


def call(method, **params):
    rpc.send_json({"method": method, "params": params})
    frames = rpc.recv_multipart()
    rep = json.loads(frames[0])
    if not rep["ok"]:
        raise RuntimeError(rep["error"])
    return rep["result"], frames[1:]


def latest(sock):
    """Most recent message: drop anything queued (streams are 'latest wins')."""
    msg = sock.recv_multipart()
    while sock.poll(0):
        msg = sock.recv_multipart()
    return msg

def select_a_goal():
    all_goals = call("get_goals")[0]
    while True:
        print("Available goals:")
        for g in all_goals:
            print(f"  - {g}")
        choice = input("Enter goal name exactly: ").strip()

        for g in all_goals:
            if choice == g['name']:
                print(f"Goal chosen: {choice}")
                return Goal(g['name'], g['pose'][0], g['pose'][1], None if len(g['pose']) <= 2 else g['pose'][2])
        print(f"{choice} is not a valid goal. Try again.\n")


# Load a scene, read the robot and camera info, save the map.
print("scenes:", [s["id"] for s in call("list_scenes")[0]])
scene, _ = call("load_scene", id=args.scene, seed=args.seed)
print("start:", scene["start"], "goals:", [g["name"] for g in scene["goals"]])
robot, _ = call("get_robot_info")
cam, _ = call("get_camera_info")
map_info, (png,) = call("get_map")
Image.open(io.BytesIO(png)).save("map.png")
print("map:", map_info)

# info from the map
map_origin = map_info['origin']
map_resolution = map_info['resolution']

# test it out
async def main():
    scene_id = args.scene
    vaccumRobot = VacuumRobot(state, cmd)
    pixelMap = PixelMap(map_origin, map_resolution, img_path="map.png", threshold=128)
    pathFinder = PathFinder(pixelMap)

    # MAIN PROGRAM PART 1: PROCESS THE MAP AND SELECT A GOAL
    selected_goal = select_a_goal()

    # MAIN PROGRAM PART 2: Kickstart the recorder
    out_dir  = f"rec/scene{scene_id}_{time.strftime('%Y%m%d-%H%M%S')}"
    recorder = RecorderService(
        call("get_camera_info"),
        out_dir,
        scene=scene_id,
        events_path=f"{out_dir}/events.jsonl",
    )
    recorder.start()
    recorder.log_event(event="run_start", scene=scene_id)

    # MAIN PROGRAM PART 3: Move robot to goal
    x, y, yaw = vaccumRobot.get_current_coordinates()
    while not selected_goal.check_within_goal(x, y, yaw):
        x, y, yaw = vaccumRobot.get_current_coordinates()
        path = pathFinder.find_best_move(x, y, selected_goal)

        for (step_x, step_y) in path.steps:
            move_status, res_x, res_y = await vaccumRobot.move_to_exact_coordinate_with_backoff(step_x, step_y, selected_goal.yaw)
            if move_status == MovementStatus.BUMPED:
                print(f"status: {move_status} at {res_x}{res_y}")
                pixelMap.add_obstacle_world_coordinates(res_x, res_y, 0.15)
                break

    call("mark_goal", name=selected_goal.name)

    # MAIN PROGRAM PART 4: stop recording
    recorder.log_event(event="run_stop", dropped=recorder.dropped)
    recorder.stop()
    print(f"[recorder] saved to {out_dir}, dropped {recorder.dropped} frames")

if __name__ == "__main__":
    asyncio.run(main())

# Camera: header, JPEG, uint16 depth (mm).
header, jpg, raw = latest(camera)
header = json.loads(header)
rgb = np.array(Image.open(io.BytesIO(jpg)))
depth = np.frombuffer(raw, "<u2").reshape(header["height"], header["width"]) * cam["depth_scale"]

# Back-project the centre pixel: pixel -> camera frame -> robot frame -> world frame.
u, v = header["width"] // 2, header["height"] // 2
z = depth[v, u]
x_camera = (u - cam["cx"]) * z / cam["fx"]
y_camera = (v - cam["cy"]) * z / cam["fy"]
p_camera = np.array([x_camera, y_camera, z, 1.0])
p_robot = np.array(cam["T_base_camera"]) @ p_camera
pose = header["pose"]
cos_yaw, sin_yaw = math.cos(pose["yaw"]), math.sin(pose["yaw"])
x_world = pose["x"] + cos_yaw * p_robot[0] - sin_yaw * p_robot[1]
y_world = pose["y"] + sin_yaw * p_robot[0] + cos_yaw * p_robot[1]
p_world = (x_world, y_world, p_robot[2])
print(f"camera seq {header['seq']}: rgb {rgb.shape}, centre depth {z:.3f} m -> world {np.round(p_world, 3)}")
