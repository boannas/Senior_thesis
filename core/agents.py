import math
import random


class Agent:
    """Base class for agents (mother and child)"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, agent_type="mother"):
        # Initialize agent position and type
        self.x, self.y = x, y 
        self.grid_w = grid_w 
        self.grid_h = grid_h 
        self.agent_type = agent_type  # "mother" or "child"

        # --- Physiological states ---
        self.hp = hp  # Health points
        self.energy = energy  # Energy level


    def move(self, dx, dy):
        if self.is_alive():
            self.x = max(0, min(self.grid_w - 1, self.x + dx))
            self.y = max(0, min(self.grid_h - 1, self.y + dy))
        else:
            pass
    
    def get_position(self):
        """Return current position as (x, y) tuple"""
        return (self.x, self.y)
    
    def Manhattan_distance_to(self, other_x, other_y):
        """Calculate Manhattan distance to another position"""
        return abs(self.x - other_x) + abs(self.y - other_y)

    def is_alive(self):
        """Check if agent is alive based on health points"""
        return self.energy > 0

    def heading_towards(self, target_x, target_y):
        """Calculate heading angle towards a target position in radians"""
        return math.atan2(target_y - self.y, target_x - self.x) * (180 / math.pi)
    
    def scan_perception(self, entities, perception_range):
        """Scan for entities within perception range"""
        perceived = []
        print(perceived)
        for entity in entities:
            dist = self.Manhattan_distance_to(entity.x, entity.y)
            deg = self.heading_towards(entity.x, entity.y)
            
        
            if dist <= perception_range and not entity.collected:
                perceived.append((entity, dist, deg))
                print(f"{self.agent_type.capitalize()} perceived {entity.name} at ({entity.x}, {entity.y}) ")
        return perceived
    
    def step_towards(self, target_x, target_y):
        """
        Move 1 step toward (target_x, target_y) using 8-direction moves.
        """
        if not self.is_alive():
            return (0, 0)

        dx = 0 if target_x == self.x else (1 if target_x > self.x else -1)
        dy = 0 if target_y == self.y else (1 if target_y > self.y else -1)
        return (dx, dy)



class MotherAgent(Agent):
    """Mother agent - can move and interact with child, food, threats, and nest"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="mother")

