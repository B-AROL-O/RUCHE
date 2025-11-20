import math
import random
import asyncio
import json

# --- LOCAL BASE CLASSES (to avoid import error) ---

class Tool:
    """Base class for MCP tools, defined locally."""
    name: str = "tool"
    description: str = "A mock tool description."
    
    async def run(self, *args, **kwargs):
        raise NotImplementedError("Tool run method must be implemented in concrete class.")

class MCPServer:
    """Base class for the MCP server, defined locally."""
    
    def __init__(self):
        self._tools = {}
        
    def register_tool(self, tool_instance: Tool):
        self._tools[tool_instance.name] = tool_instance
        print(f"INFO: Mock Tool registered: {tool_instance.name}")
        
    def start(self):
        # This mock method is not called via asyncio.run
        print("INFO: MCP Server Mock Started successfully.")
        pass

# State and Logic
class Robot:
    """Manages the robot's internal state and movement logic."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0  # degrees: 0=East, 90=North, 180=West, 270=South
        self.distance_traveled = 0.0
        self.left_turns = 0
        self.right_turns = 0

    def get_state(self) -> dict:
        """Returns the current robot telemetry rounded for output."""
        return {
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "heading": self.heading,
            "distance_traveled": round(self.distance_traveled, 2),
            "left_turns": self.left_turns,
            "right_turns": self.right_turns
        }

    async def move_forward(self, distance: float) -> dict:
        """Moves the robot forward based on the current heading."""
        if distance <= 0:
            raise ValueError("Distance must be a positive float.")

        rad_heading = math.radians(self.heading)
        mock_error_factor = random.uniform(0.95, 1.05)  # Simulates ±5% error

        # Calculate movement components
        dx = distance * math.cos(rad_heading) * mock_error_factor
        dy = distance * math.sin(rad_heading) * mock_error_factor
        
        # Simulate execution time
        await asyncio.sleep(distance * 0.05) 

        self.x += dx
        self.y += dy
        self.distance_traveled += distance

        return {
            "status": "success",
            "action": "moved_forward",
            "distance_requested": distance,
            "current_state": self.get_state()
        }

    async def turn_left(self) -> dict:
        """Turns the robot 90 degrees left."""
        self.heading = (self.heading + 90) % 360
        self.left_turns += 1
        await asyncio.sleep(0.1)
        
        return {
            "status": "success",
            "action": "turned_left",
            "current_state": self.get_state()
        }

    async def turn_right(self) -> dict:
        """Turns the robot 90 degrees right."""
        self.heading = (self.heading + 270) % 360
        self.right_turns += 1
        await asyncio.sleep(0.1)
        
        return {
            "status": "success",
            "action": "turned_right",
            "current_state": self.get_state()
        }


# --- Robot Instance ---
robot_instance = Robot()

# --- MCP Tools ---

class MoveForward(Tool):
    name = "move_forward"
    description = "Moves the robot forward by N meters (positive float). Use to change position (x, y)."
    
    async def run(self, distance: float):
        return await robot_instance.move_forward(distance)

class TurnLeft(Tool):
    name = "turn_left"
    description = "Turns the robot 90 degrees left. Use to change the 'heading' counter-clockwise."
    
    async def run(self):
        return await robot_instance.turn_left()

class TurnRight(Tool):
    name = "turn_right"
    description = "Turns the robot 90 degrees right. Use to change the 'heading' clockwise."
    
    async def run(self):
        return await robot_instance.turn_right()

class GetTelemetry(Tool):
    name = "get_telemetry"
    description = "Returns the current robot state (x, y, heading, distance, turns). Use this to check the robot's current location and direction."
    
    async def run(self):
        return robot_instance.get_state()

# This block only runs to print registrations, but does not start anything
if __name__ == "__main__":
    server = MCPServer()
    server.register_tool(MoveForward())
    server.register_tool(TurnLeft())
    server.register_tool(TurnRight())
    server.register_tool(GetTelemetry())

    print("--- MCP Server Initialization Complete ---")
