import random
from core.agents import MotherAgent, ChildAgent, ThreatAgent
from core.entities import Food, Nest

class World:
    def __init__(self, grid_w, grid_h, mother_start=None, child_start=None, 
                 food_positions=None, threat_positions=None, nest_position=None, seed=42):
        random.seed(seed)
        self.t = 0
        self.grid_w = grid_w
        self.grid_h = grid_h
        
        # Initialize mother agent
        if mother_start is None:
            mx, my = grid_w // 2, grid_h // 2 
        else:
            mx, my = mother_start
        
        self.mother = MotherAgent(mx, my, grid_w, grid_h, hp=100, energy=100)
        
        # Initialize child agent
        if child_start is None:
            cx, cy = grid_w // 2 + 1, grid_h // 2
        else:
            cx, cy = child_start
        self.child = ChildAgent(cx, cy, grid_w, grid_h, hp=100, energy=100)
        
        # Initialize food
        self.foods = []
        if food_positions:
            for fx, fy in food_positions:
                self.foods.append(Food(fx, fy))
        
        # Initialize threats
        self.threats = []
        if threat_positions:
            for tx, ty in threat_positions:
                self.threats.append(ThreatAgent(tx, ty, grid_w, grid_h, hp=100, energy=100))
        
        # Initialize nest
        self.nest = None
        if nest_position:
            nx, ny = nest_position
            self.nest = Nest(nx, ny)

    def step(self, mother_action: int, child_action: int = None):
        """
        Step the world forward
        Args:
            mother_action: Action for mother agent (0=stay, 1=up, 2=down, 3=left, 4=right)
            child_action: Optional action for child agent (if not being carried)
        """
        moves = {0: (0, 0), 1: (0, -1), 2: (0, 1), 3: (-1, 0), 4: (1, 0)}
        
        # Move child first if not being carried
        if not self.child.is_carried:
            if child_action is not None:
                cdx, cdy = moves.get(child_action, (0, 0))
                self.child.move(cdx, cdy)
        
        # Move mother
        dx, dy = moves.get(mother_action, (0, 0))
        self.mother.move(dx, dy)
        print(self.mother.scan_perception(self.foods + self.threats, perception_range=3))
        # print((self.mother.energy))
        
        # If child is carried, move child with mother
        if self.child.is_carried:
            self.child.x = self.mother.x
            self.child.y = self.mother.y
        
        # Move threats randomly
        for threat in self.threats:
            if threat.active:
                threat.random_move()
        
        # Check interactions after movement
        self._check_interactions()
        
        self.t += 1
    
    def _check_interactions(self):
        """Check and handle interactions between entities"""
        mx, my = self.mother.x, self.mother.y
        cx, cy = self.child.x, self.child.y
        
        # Check if mother picks up child (same position and child not carried)
        if not self.mother.has_child and not self.child.is_carried:
            if mx == cx and my == cy:
                self.mother.has_child = True
                self.child.is_carried = True
        
        # Check if mother drops child at nest
        if self.mother.has_child and self.nest:
            if mx == self.nest.x and my == self.nest.y:
                self.mother.has_child = False
                self.child.is_carried = False
                self.child.x = mx
                self.child.y = my
        
        # Check if mother collects food
        if not self.mother.has_food:
            for food in self.foods:
                if not food.collected and mx == food.x and my == food.y:
                    food.collect()
                    self.mother.has_food = True
                    break
        
        # Check if mother delivers food to nest
        if self.mother.has_food and self.nest:
            if mx == self.nest.x and my == self.nest.y:
                self.mother.has_food = False
        
        # Check if child encounters threat
        for threat in self.threats:
            if threat.active and cx == threat.x and cy == threat.y:
                # Threat affects child (you can add logic here)
                pass
        
        # Check if mother encounters threat
        for threat in self.threats:
            if threat.active and mx == threat.x and my == threat.y:
                # Threat affects mother (you can add logic here)
                pass
