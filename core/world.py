import random
from core import agents
from core.agents import MotherAgent
from core.entities import Food

class World:
    def __init__(self, grid_w, grid_h, mother_starts=None, child_start=None, 
                 food_positions=None, threat_positions=None, nest_position=None, seed=42):
        
        # Initialize mother agents
        self.mothers = []
        for (mx, my) in mother_starts:
            self.mothers.append(MotherAgent(mx, my, grid_w, grid_h, hp=100, energy=100))

        random.seed(seed)
        self.t = 0
        self.grid_w = grid_w
        self.grid_h = grid_h
        
        # Initialize food
        self.foods = []
        if food_positions:
            for fx, fy in food_positions:
                self.foods.append(Food(fx, fy))

    def mother_decision(self):
        perception_range = 100 
        proposals = {}

        for mother in self.mothers:
            perceived = mother.scan_perception(self.foods, perception_range=perception_range)

            if not perceived:
                dx, dy = mother.step_towards(4,4)
                nx = max(0, min(self.grid_w - 1, mother.x + dx))
                ny = max(0, min(self.grid_h - 1, mother.y + dy))
                proposals[mother] = (nx, ny)
                
                continue

            perceived.sort(key=lambda t: t[1])

            target_food = perceived[0][0]   # nearest food
            dx, dy = mother.step_towards(target_food.x, target_food.y)
            nx = max(0, min(self.grid_w - 1, mother.x + dx))
            ny = max(0, min(self.grid_h - 1, mother.y + dy))
            proposals[mother] = (nx, ny)

            # Check if food is collected (distance == 0)
            if perceived[0][1] == 0 and not target_food.collected: 
                target_food.collect()
            

        self.resolve_and_apply_moves(self.mothers, proposals)

    def resolve_and_apply_moves(self, agents, proposals):
        """""Resolve movement proposals to avoid collisions and apply valid moves."""
        current = {a: (a.x, a.y) for a in agents}

        dest_map = {}
        for a, dest in proposals.items():
            dest_map.setdefault(dest, []).append(a)

        blocked = set()
        for dest, claimers in dest_map.items():
            if len(claimers) > 1:
                winner = random.choice(claimers)    # Can change to deterministic priority later
                for a in claimers:
                    if a is not winner:
                        blocked.add(a)
                # blocked.update(claimers)

        # block swaps
        for a, dest_a in proposals.items():
            if a in blocked:
                continue
            for b, dest_b in proposals.items():
                if b is a or b in blocked:
                    continue
                if dest_a == current[b] and dest_b == current[a]:
                    blocked.add(a)
                    blocked.add(b)

        occupied = set(current.values())
        for a, (nx, ny) in proposals.items():
            if a in blocked:
                continue
            if (nx, ny) in occupied and (nx, ny) != current[a]:
                continue
            occupied.discard(current[a])
            a.x, a.y = nx, ny
            occupied.add((nx, ny))


    def step(self, dt: float = 0.1):
        """
        Advance world by one simulation timestep dt.
        """
        self.mother_decision()
        self.t += dt
        
        
