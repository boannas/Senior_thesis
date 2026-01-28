import random
from core.agents import ChildAgent, MotherAgent, ThreatAgent
from core.entities import Food
from func.function_code import astar

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
        self.RANDOM_MOVES = [(0,0), (1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

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
    # def mother_decision(self):
    #     perception_range = 100 
    #     proposals = {}
    #     occupied_now = {(m.x, m.y) for m in self.mothers}

    #     # Entities and agent - Mother received (All)
    #     mother_receive = self.foods + self.mothers + self.children + self.threats

    #     for mother in self.mothers:
    #         food_perceived, agents_perceived = mother.scan_perception(mother_receive, perception_range=perception_range)

    #         if not food_perceived:
    #             child = mother.child
                
    #             if child is not None:
    #                 goal = (child.x, child.y)

    #                 dx, dy = mother.step_towards(*goal)

    #                 nx = max(0, min(self.grid_w - 1, mother.x + dx))
    #                 ny = max(0, min(self.grid_h - 1, mother.y + dy))

    #                 # if blocked right now, locally replan one step
    #                 if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
    #                     nx, ny = self.best_step(mother, goal, occupied_now)
    #                     # print(mother.id , nx, ny)
    #                     # print(mother)
    #                 proposals[mother] = (nx, ny)

    #                 if child is not None and abs(nx - child.x) + abs(ny - child.y) == 0:
    #                     child.set_carried(True)
    #             continue

    #         food_perceived.sort(key=lambda t: t[1])

    #         target_food = food_perceived[0][0]   # nearest food
    #         goal = (target_food.x, target_food.y)
    #         dx, dy = mother.step_towards(*goal)
    #         nx = max(0, min(self.grid_w - 1, mother.x + dx))
    #         ny = max(0, min(self.grid_h - 1, mother.y + dy))
    #         # if blocked right now, locally replan one step
    #         if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
    #             nx, ny = self.best_step(mother, goal, occupied_now)

    #         proposals[mother] = (nx, ny)


    #         # Check if food is collected (distance == 0)
    #         if abs(nx - target_food.x) + abs(ny - target_food.y) == 0 and not target_food.collected:
    #             target_food.collect()


    #     self.resolve_and_apply_moves(self.mothers, proposal
    
    # def mother_decision(self):
    #     perception_range = 100
    #     proposals = {}
    #     occupied_now = {(m.x, m.y) for m in self.mothers}

    #     mother_receive = self.foods + self.mothers + self.children + self.threats

    #     for mother in self.mothers:
    #         food_perceived, agents_perceived = mother.scan_perception(
    #             mother_receive, perception_range=perception_range
    #         )

    #         # choose goal
    #         goal = None
    #         target_food = None

    #         if food_perceived:
    #             food_perceived.sort(key=lambda t: t[1])
    #             target_food = food_perceived[0][0]
    #             goal = (target_food.x, target_food.y)
    #         else:
    #             child = mother.child
    #             if child is None:
    #                 proposals[mother] = (mother.x, mother.y)
    #                 continue
    #             if child.is_carried:
    #                 proposals[mother] = (mother.x, mother.y)
    #                 child.x, child.y = mother.x, mother.y
    #                 continue
    #             goal = (child.x, child.y)

    #         if goal is None:
    #             # nx, ny = mother.x, mother.y
    #             continue

    #         # dynamic obstacles: other mothers (not self)
    #         blocked = {(m.x, m.y) for m in self.mothers if m is not mother}

    #         path = astar(
    #             start=(mother.x, mother.y),
    #             goal=goal,
    #             grid_w=self.grid_w,
    #             grid_h=self.grid_h,
    #             blocked=blocked,
    #             moves_8=True
    #         )
    #         print(path)
    #         if path is None or len(path) < 2:
    #             # fallback: your local best_step
    #             nx, ny = self.best_step(mother, goal, occupied_now)
    #         else:
    #             nx, ny = path[1]  # next step

    #             # if next step is occupied RIGHT NOW (rare due to blocked), fallback
    #             if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
    #                 nx, ny = self.best_step(mother, goal, occupied_now)

    #         proposals[mother] = (nx, ny)
    #         child = mother.child

    #         # if reached food -> collect
    #         if target_food is not None and (nx, ny) == (target_food.x, target_food.y) and not target_food.collected:
    #             target_food.collect()

    #         # if reached child -> carry
    #         elif child is not None and (nx, ny) == (child.x, child.y) and not food_perceived:
    #             child.set_carried(True)
    #             # child.x, child.y = nx, ny

            

    #     self.resolve_and_apply_moves(self.mothers, proposals)

    def mother_decision(self):
        perception_range = 100
        proposals = {}
        occupied_now = {(m.x, m.y) for m in self.mothers}
        mother_receive = self.foods + self.mothers + self.children + self.threats

        # store intended target per mother (optional but helpful)
        intended_food = {}   # mother -> Food
        intended_child = set()

        for mother in self.mothers:
            food_perceived, _ = mother.scan_perception(mother_receive, perception_range=perception_range)

            goal = None
            target_food = None
            child = mother.child

            if food_perceived and not mother.pick_food:
                food_perceived.sort(key=lambda t: t[1])
                target_food = food_perceived[0][0]
                goal = (target_food.x, target_food.y)
                intended_food[mother] = target_food
            else:
                if child is None or child.is_carried:
                    proposals[mother] = (mother.x, mother.y)
                    # goal = (0.0, 0.0)
                    continue
                elif child.is_alive():
                    goal = (child.x, child.y)
                    intended_child.add(mother)

            # print(mother.id, goal)

            if goal is None:
                valid_moves = []
                # proposals[mother] = (mother.x, mother.y)
                for dx, dy in self.RANDOM_MOVES:
                    nx, ny = mother.x + dx, mother.y + dy
                    if self.in_bounds(nx, ny):
                        valid_moves.append((nx, ny))

                proposals[mother] = random.choice(valid_moves) if valid_moves else (mother.x, mother.y)
                # print(proposals[mother])
                continue

            # BLOCK other mothers + other children (prevents camping on чужие дети)
            blocked = {(m.x, m.y) for m in self.mothers if m is not mother}
            blocked |= {(c.x, c.y) for c in self.children
                        if (c is not mother.child) and (not c.is_carried)}

            path = astar((mother.x, mother.y), goal, self.grid_w, self.grid_h, blocked, moves_8=True)

            if path is None or len(path) < 2:
                nx, ny = self.best_step(mother, goal, occupied_now)
            else:
                nx, ny = path[1]
                if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
                    nx, ny = self.best_step(mother, goal, occupied_now)

            proposals[mother] = (nx, ny)

        # 1) apply moves
        self.resolve_and_apply_moves(self.mothers, proposals)

        # 2) now apply interactions using REAL positions
        for mother in self.mothers:
            # food collect
            f = intended_food.get(mother)
            if f is not None and (mother.x, mother.y) == (f.x, f.y) and not f.collected:
                f.collect()
                mother.picking_food(True)

            # child pickup (only if this mother was going-to-child)
            child = mother.child
            if mother in intended_child and child is not None and (mother.x, mother.y) == (child.x, child.y) and child.is_alive():
                child.set_carried(True)

            # sync carried child position always
            if child is not None and child.is_carried:
                child.x, child.y = mother.x, mother.y
        # print(mother.n_foods)

    def resolve_and_apply_moves(self, agents, proposals):
        current = {a: (a.x, a.y) for a in agents}

        def prio(a):
            # prefer explicit IDs when present; fallback to object id
            v = getattr(a, "id", None)
            if v is None:
                v = getattr(a, "mother_id", None)
            if v is None:
                v = getattr(a, "child_id", None)
            if v is None:
                return id(a)
            return v

        dest_map = {}
        for a, dest in proposals.items():
            dest_map.setdefault(dest, []).append(a)

        blocked = set()

        # 1) destination conflicts
        for dest, claimers in dest_map.items():
            if len(claimers) > 1:
                winner = min(claimers, key=prio)
                for a in claimers:
                    if a is not winner:
                        blocked.add(a)

        # 2) swap conflicts (yield deterministically)
        for a, dest_a in proposals.items():
            if a in blocked:
                continue
            for b, dest_b in proposals.items():
                if b is a or b in blocked:
                    continue
                if dest_a == current[b] and dest_b == current[a]:
                    winner = min([a, b], key=prio)
                    loser = b if winner is a else a
                    blocked.add(loser)

        # 3) apply moves
        occupied = set(current.values())
        for a, (nx, ny) in proposals.items():
            if a in blocked:
                continue

            nx = max(0, min(self.grid_w - 1, nx))
            ny = max(0, min(self.grid_h - 1, ny))
            
            if (nx, ny) in occupied and (nx, ny) != current[a]:
                continue
            occupied.discard(current[a])
            a.x, a.y = nx, ny
            occupied.add((nx, ny))


    def threat_decision(self):
        perception_range = 2 
        avoid_range = 2
        proposals = {}

        # Entities and agent - Threat received (mother + child) not care other threats & food  
        threat_receive = self.mothers + self.children

        for threat in self.threats:
            _, agents_perceived = threat.scan_perception(threat_receive, perception_range)
            mother_percieved = agents_perceived[0]
            child_perceived = agents_perceived[1]

            # # 1st priority: Afraid to mother agent (Moving away from mother)
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
                    # continue

            # 2nd priority: Child huntering (Moving closer to closest child)
            elif child_perceived:
                target_child, _ = child_perceived[0]
                if not target_child.is_carried:
                    dx, dy = threat.step_towards(target_child.x, target_child.y)
                    nx = max(0, min(self.grid_w - 1, threat.x + dx))
                    ny = max(0, min(self.grid_h - 1, threat.y + dy))
                    proposals[threat] = (nx, ny)
                
                dist = abs(target_child.x - threat.x) + abs(target_child.y - threat.y)
                # print([threat.x, threat.y], [target_child.x, target_child.y])
                # print(target_child, target_child.energy, dist)
                # Attack if on child
                if dist == 0 and target_child.is_alive():
                    target_child.energy -= random.randint(5, 10)

            # last choice: Threat move randomly
            else:
                dx, dy = random.choice(self.RANDOM_MOVES)
                nx = max(0, min(self.grid_w - 1, threat.x + dx))
                ny = max(0, min(self.grid_h - 1, threat.y + dy))
                proposals[threat] = (nx, ny)
                # continue

            

        self.resolve_and_apply_moves(self.threats, proposals)

    def best_step(self, mother, goal_xy, occupied_now):
        gx, gy = goal_xy
        candidates = [(mother.x, mother.y)]
        for dx, dy in self.RANDOM_MOVES:
            nx, ny = mother.x + dx, mother.y + dy
            if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                candidates.append((nx, ny))

        def score(cell):
            nx, ny = cell

            # Check occupied grid
            if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
                return 10**9

            s = abs(nx - gx) + abs(ny - gy)     # Distance from possible action <-> goal point

            for (ox, oy) in occupied_now:
                # print(occupied_now)
                if (ox, oy) == (mother.x, mother.y):   
                    continue

                # Checking not to move closer to another mother 
                if abs(nx - ox) + abs(ny - oy) == 1:
                    # print([nx, ny], [ox, oy])
                    s += 0

            # Stay the same place will be small pernalty
            if (nx, ny) == (mother.x, mother.y):
                s += 10**9
            return s
       
        # print([mother.x, mother.y], candidates , min(candidates))
        return min(candidates, key=score)

    def in_bounds(self, x, y):
        return 0 <= x < self.grid_w and 0 <= y < self.grid_h

