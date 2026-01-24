import random
from core.agents import MotherAgent
from core.entities import Food

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
        
        # Initialize food
        self.foods = []
        if food_positions:
            for fx, fy in food_positions:
                self.foods.append(Food(fx, fy))


    def mother_decision(self):
        # Recieve perceived of `Mother agent`
        perception_range = 100 
        perceived = self.mother.scan_perception(self.foods, perception_range=perception_range)

        # Decision-making for Mother agent
        if perceived:
            # choose nearest food (min Manhattan distance)
            target_food, dist, deg = min(perceived, key=lambda t: t[1])

            # Move towards target food
            dx, dy = self.mother.step_towards(target_food.x, target_food.y)
            self.mother.move(dx, dy)

            if dist == 0 and not target_food.collected:
                target_food.collect()
                print(f"Food at ({target_food.x}, {target_food.y}) collected!")

        else:
            print("Mother perceives no food within range.")
    

    def step(self, mother_action: int, child_action: int = None, dt: float = 0.1):
        """
        Advance world by one simulation timestep dt.
        """
        self.mother_decision()
        self._check_interactions()
        self.t += dt

    def _check_interactions(self):
        """Check and handle interactions between entities"""
        mx, my = self.mother.x, self.mother.y
        
        
        
