import math
import random

from core.entities import Food


class Agent:
    """Base class for agents (mother and child)"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, agent_type="mother"):
        # Initialize agent position and type
        self.x, self.y = x, y 
        self.grid_w = grid_w 
        self.grid_h = grid_h 
        self.agent_type = agent_type  # "mother" or "child"

        # --- Physiological states ---
        self.energy = energy  # Energy level

    def move(self, dx, dy):
        if self.is_alive():
            self.x = max(0, min(self.grid_w - 1, self.x + dx))
            self.y = max(0, min(self.grid_h - 1, self.y + dy))
        else:
            pass
    
    # def get_position(self):
    #     """Return current position as (x, y) tuple"""
    #     return (self.x, self.y)
    
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
        food_perceived = []
        mother = []
        child = []
        threat = []
        agents_perceived = []
        

        for e in entities:
            if e is self:   # Skip self
                continue

            dist = self.Manhattan_distance_to(e.x, e.y)
            if dist > perception_range: # Out of range
                continue

            # Food
            if isinstance(e, Food):
                if getattr(e, "collected", False):
                    continue
                food_perceived.append((e, dist))
                continue

            # Agents (other mothers, child, threats)
            if isinstance(e, Agent):
                if e.agent_type == 'mother':
                    mother.append((e, dist))
                elif e.agent_type == 'child' and e.is_alive():
                    # print(e.id , e.energy)
                    child.append((e, dist))
                elif e.agent_type == 'threat':
                    threat.append((e, dist))
                else:
                    pass
                continue
        agents_perceived = [mother, child, threat]        
        return food_perceived, agents_perceived

    
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
    def __init__(self, x, y, grid_w, grid_h, hp, energy, mother_id=None):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="mother")
        self.id = mother_id
        self.child = None  # Reference to carried child agent

    def set_child(self, child_agent):
        """Set the child agent being carried by mother"""
        self.child = child_agent


class ChildAgent(Agent):
    """Child agent - can be carried by mother"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, child_id=None):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="child")
        self.id = child_id
        self.mother = None  # Reference to mother agent
        self.is_carried = False
        self.distress = 0

    def set_mother(self, mother_agent):
        """Set the mother agent carrying this child"""
        self.mother = mother_agent

    def set_carried(self, carried: bool):
        """Set whether the child is being carried by mother"""
        self.is_carried = carried

class ThreatAgent(Agent):
    """Threat agent - can move and pose danger to mother and child"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, threat_id=None):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="threat")
        self.id = threat_id

