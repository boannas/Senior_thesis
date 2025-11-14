import math
import random


class Agent:
    """Base agent class for Mother and Child agents"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, agent_type="mother"):
        # Initialize agent position and type
        self.x, self.y = x, y
        self.grid_w = grid_w
        self.grid_h = grid_h
        self.agent_type = agent_type  # "mother" or "child"

        # --- General states (fixed gene)---
        self.altruism_level = 0.0
        self.kin_bias = 0.0
        self.nurture_threshold = 0.0 
        self.threat_sensitivity = 0.0

        # --- Physiological states ---
        self.hp = hp  # Health points
        self.energy = energy  # Energy level
        self.hunger = 0.0
        self.fatigue = 0.0

        # --- Neuroendocrine states ---
        self.oxytocin = 0.0
        self.vasopressin = 0.0
        self.dopamine = 0.0
        self.cortisol = 0.0
        self.maternal_bond = 0.0

    def move(self, dx, dy):
        self.x = max(0, min(self.grid_w - 1, self.x + dx))
        self.y = max(0, min(self.grid_h - 1, self.y + dy))

        # Decay energy on move
        # self.energy -= self.decay_rate
        # if self.energy <= 0:
        #     self.alive = False  
    
    def get_position(self):
        return (self.x, self.y)
    
    def distance_to(self, other_x, other_y):
        """Calculate Euclidean distance to another position"""
        return ((self.x - other_x) ** 2 + (self.y - other_y) ** 2) ** 0.5
    
    def Manhattan_distance_to(self, other_x, other_y):
        """Calculate Manhattan distance to another position"""
        return abs(self.x - other_x) + abs(self.y - other_y)

    def heading_towards(self, target_x, target_y):
        """Calculate heading angle towards a target position in radians"""
        return math.atan2(target_y - self.y, target_x - self.x) * (180 / math.pi)
    
    def scan_perception(self, entities, perception_range):
        """Scan for entities within perception range"""
        perceived = []
        for entity in entities:
            dist = self.Manhattan_distance_to(entity.x, entity.y)
            deg = self.heading_towards(entity.x, entity.y)
            if dist <= perception_range:
                perceived.append((entity, dist, deg))
        return perceived


class MotherAgent(Agent):
    """Mother agent - can move and interact with child, food, threats, and nest"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="mother")
        self.has_child = False  # Whether mother is carrying child
        self.has_food = False   # Whether mother is carrying food


class ChildAgent(Agent):
    """Child agent - can move independently or be carried by mother"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="child")
        self.is_carried = False  # Whether child is being carried by mother


class ThreatAgent(Agent):
    """Threat entity that agents should avoid"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="threat")
        self.active = True  # Whether threat is active
        
    def is_at_position(self, x, y):
        """Check if threat is at given position"""
        return self.x == x and self.y == y
    
    def random_move(self):
        """Move the threat agent randomly in one of four directions"""
        # 0=stay, 1=up, 2=down, 3=left, 4=right
        moves = {0: (0, 0), 1: (0, -1), 2: (0, 1), 3: (-1, 0), 4: (1, 0)}
        # Choose a random action (including stay)
        action = random.randint(0, 4)
        dx, dy = moves[action]
        self.move(dx, dy)