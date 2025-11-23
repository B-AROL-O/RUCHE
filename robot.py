import math
import random
import asyncio

class Robot:
    """Manages the robot's internal state and movement logic."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0  # 0=East, 90=North, etc.
        self.distance_traveled = 0.0
        self.left_turns = 0
        self.right_turns = 0

    def get_state(self) -> dict:
        return {
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "heading": self.heading,
            "distance_traveled": round(self.distance_traveled, 2),
            "left_turns": self.left_turns,
            "right_turns": self.right_turns,
        }

    async def move_forward(self, distance: float) -> dict:
        if distance <= 0:
            raise ValueError("Distance must be positive.")

        rad = math.radians(self.heading)
        noise = random.uniform(0.95, 1.05)

        dx = distance * math.cos(rad) * noise
        dy = distance * math.sin(rad) * noise
        
        await asyncio.sleep(distance * 0.05)

        self.x += dx
        self.y += dy
        self.distance_traveled += distance

        return {
            "action": "move_forward",
            "distance_requested": distance,
            "state": self.get_state(),
        }

    async def turn_left(self) -> dict:
        self.heading = (self.heading + 90) % 360
        self.left_turns += 1
        await asyncio.sleep(0.1)
        return {"action": "turn_left", "state": self.get_state()}

    async def turn_right(self) -> dict:
        self.heading = (self.heading - 90) % 360
        self.right_turns += 1
        await asyncio.sleep(0.1)
        return {"action": "turn_right", "state": self.get_state()}


# Global instance used everywhere
robot_instance = Robot()
