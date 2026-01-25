import random
# from core import agents
from core.agents import ChildAgent, MotherAgent, ThreatAgent
from core.entities import Food

class World:
    def __init__(self, grid_w, grid_h, mother_starts=None, child_start=None, 
                 food_positions=None, threat_starts=None, seed=42):
        # Fixed seed for reproducibility
        random.seed(seed)
        self.t = 0
        self.grid_w = grid_w
        self.grid_h = grid_h

        # Initialize mother agents
        self.mothers = []
        self.children = []
        self.threats = []

        # 8-directions + stay
        self.RANDOM_MOVES = [(0,0),(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]


        # Create mother agents
        for i, (mx, my) in enumerate(mother_starts):
            self.mothers.append(MotherAgent(mx, my, grid_w, grid_h, hp=100, energy=100, mother_id=f"M{i}"))

        # Create child agents
        for i, (cx, cy) in enumerate(child_start):
            self.children.append(ChildAgent(cx, cy, grid_w, grid_h, hp=50, energy=50, child_id=f"C{i}"))

        # Link mothers and children (now with like with index)
        for m, c in zip(self.mothers, self.children):
            m.set_child(c)
            c.set_mother(m)
        
        # Initialize food
        self.foods = []
        if food_positions:
            for (fx, fy) in food_positions:
                self.foods.append(Food(fx, fy))

        # Create Threat
        for i, (cx, cy) in enumerate(threat_starts):
            self.threats.append(ThreatAgent(cx, cy, grid_w, grid_h, hp=50, energy=50))

    def step(self, dt: float = 0.1):
        """
        Advance world by one simulation timestep dt.
        """
        self.mother_decision()
        self.threat_decision()
        self.t += dt
        



# ------------- Mother Decision ---------------
    def mother_decision(self):
        perception_range = 100 
        proposals = {}
        # dx, dy = 0, 0

        for mother in self.mothers:
            food_perceived, agents_perceived = mother.scan_perception(self.foods + self.mothers + self.children + self.threats, perception_range=perception_range)

            # print("mother", agents_perceived[0], '\n')    
            # print("child", agents_perceived[1], '\n')    
            # print("threat", agents_perceived[2], '\n')    

            if not food_perceived:
                child = mother.child
                
                if child is not None:
                    dx, dy = mother.step_towards(child.x, child.y)

                    nx = max(0, min(self.grid_w - 1, mother.x + dx))
                    ny = max(0, min(self.grid_h - 1, mother.y + dy))
                    proposals[mother] = (nx, ny)

                    if child is not None and mother.Manhattan_distance_to(child.x, child.y) == 0:
                        # Pick up child
                        child.set_carried(True)
                continue

            food_perceived.sort(key=lambda t: t[1])

            target_food = food_perceived[0][0]   # nearest food
            dx, dy = mother.step_towards(target_food.x, target_food.y)
            nx = max(0, min(self.grid_w - 1, mother.x + dx))
            ny = max(0, min(self.grid_h - 1, mother.y + dy))
            proposals[mother] = (nx, ny)

            # Check if food is collected (distance == 0)
            if food_perceived[0][1] == 0 and not target_food.collected: 
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
        
    def threat_decision(self):
        perception_range = 2 
        avoid_range = 0
        proposals = {}

        for threat in self.threats:
            _, agents_perceived = threat.scan_perception(self.mothers + self.children, perception_range)
            mother_percieved = agents_perceived[0]
            child_perceived = agents_perceived[1]

            if mother_percieved:
                mother_percieved.sort(key=lambda t:t[1])
                closest_mother, m_dist = mother_percieved[0]

                if m_dist <= avoid_range:
                    best_moves = []
                    best_dist = -1

                    for (dx, dy) in self.RANDOM_MOVES:
                        nx = max(0, min(self.grid_w - 1, threat.x + dx))
                        ny = max(0, min(self.grid_h - 1, threat.y + dy))
                        new_dist = abs(nx - closest_mother.x) + abs(ny - closest_mother.y)

                        if new_dist > best_dist:
                            best_dist = new_dist
                            best_moves = [(nx, ny)]
                        elif new_dist == best_dist:
                            best_moves.append((nx, ny))
                    # print(best_moves)

                    proposals[threat] = random.choice(best_moves)
                    continue

            elif not child_perceived:
                dx, dy = random.choice(self.RANDOM_MOVES)
                nx = max(0, min(self.grid_w - 1, threat.x + dx))
                ny = max(0, min(self.grid_h - 1, threat.y + dy))
                proposals[threat] = (nx, ny)
                continue

            target_child, dist = child_perceived[0]
            if not target_child.is_carried:
                dx, dy = threat.step_towards(target_child.x, target_child.y)
                nx = max(0, min(self.grid_w - 1, threat.x + dx))
                ny = max(0, min(self.grid_h - 1, threat.y + dy))
                proposals[threat] = (nx, ny)
            else:
                # dx, dy = random.choice(self.RANDOM_MOVES)
                pass



            # Attack if on child
            if dist == 0 and target_child.is_alive():
                target_child.energy -= random.randint(5, 10)

        self.resolve_and_apply_moves(self.threats, proposals)


