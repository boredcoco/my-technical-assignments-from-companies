import json
import math
import time
import asyncio
from typing import Optional

from .MovementStatus import MovementStatus


class VacuumRobot:
    def __init__(self, state, cmd):
        self.state = state
        self.cmd = cmd
        self.robot_state = json.loads(self.latest(self.state)[0])
        self.pose = self.robot_state["pose"]
        self.cmd_id = 1
        status_string = (self.robot_state.get("goto") or {}).get("status", "")

        goto_info = self.robot_state.get("goto") or {}
        self.status = self.parse_status(goto_info.get("status", ""))
        self.current_status_id = goto_info.get("id")

    def get_current_coordinates(self):
        self.update_robot_state()
        return (self.pose['x'], self.pose['y'], self.pose['yaw'])

    def update_robot_state(self) -> None:
        buf = self.latest(self.state)
        if not buf:
            return
        self.robot_state = json.loads(buf[0])
        self.pose = self.robot_state["pose"]
        goto_info = self.robot_state.get("goto") or {}
        self.status = MovementStatus.parse(goto_info.get("status"))
        self.current_status_id = goto_info.get("id")     # ← track the id

    async def move_to_exact_coordinate_with_backoff(self, x, y, yaw):
        # function should return (status, x, y) where x y are the coordinates of the bump
        initial_status = await self.move_to_exact_coordinate(x, y, yaw)
        current_coordinates = (self.pose['x'], self.pose['y'])
        if initial_status == MovementStatus.BUMPED:
            backoff_status = await self.cancel_latest_cmd_and_back_off(10)
            return (backoff_status, current_coordinates[0], current_coordinates[1])
        return (initial_status, current_coordinates[0], current_coordinates[1])

    async def move_to_exact_coordinate(self, x, y, yaw) -> MovementStatus:
        # if current position is within 5cm do not execute move
        if abs(self.pose['x'] - x) < 0.05 and abs(self.pose['y'] - y) < 0.05:
            return MovementStatus.REACHED

        this_id = self.cmd_id
        self.cmd_id += 1
        self.cmd.send_json({"type": "goto", "id": this_id, "x": x, "y": y, "yaw": yaw})

        self.status = MovementStatus.UNKNOWN
        self.current_status_id = None                    # ← reset, so old status can't match

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self.update_robot_state()
            if self.current_status_id == this_id and self.status.is_terminal:
                return self.status                       # ← only trust status for OUR id
            await asyncio.sleep(0.02)
        return self.status


    async def cancel_latest_cmd_and_back_off(self, backoff_amt) -> MovementStatus:
    # 1. Cancel in-flight move
        last_id = self.cmd_id - 1
        self.cmd.send_json({"type": "cancel", "id": last_id})
        await asyncio.sleep(0.3)

        # 2. Register obstacle so BFS avoids this wall next attempt
        bump_x = self.pose["x"] + 0.15 * math.cos(self.pose["yaw"])
        bump_y = self.pose["y"] + 0.15 * math.sin(self.pose["yaw"])

        # 3. Drive backward: re-send the velocity command at ~20 Hz for 1 second
        start_x, start_y = self.pose["x"], self.pose["y"]

        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            self.cmd.send_json({"type": "wheels", "left": -backoff_amt, "right": -backoff_amt})
            await asyncio.sleep(0.05)          # 50ms = 20 Hz stream
            self.update_robot_state()          # keep pose fresh during the drive

        self.cmd.send_json({"type": "wheels", "left": 0.0, "right": 0.0})
        await asyncio.sleep(0.2)
        self.update_robot_state()

        dist = math.hypot(self.pose["x"] - start_x, self.pose["y"] - start_y)
        return MovementStatus.BUMPED

        # self.cmd.send_json({"type": "wheels", "left": -2.0, "right": -2.0})
        # await asyncio.sleep(0.5)
        # self.cmd.send_json({"type": "wheels", "left": 0.0, "right": 0.0})
        # await asyncio.sleep(0.2)

        # self.update_robot_state()
        # return self.status

    @staticmethod
    def parse_status(status) -> MovementStatus:
        return MovementStatus.parse(status)

    def latest(self, sock):
        msg = sock.recv_multipart()
        while sock.poll(0):
            msg = sock.recv_multipart()
        return msg
