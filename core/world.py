import random
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
        self.RANDOM_MOVES = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

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
    
    def mother_decision(self):
        perception_range = 100
        proposals = {}
        occupied_now = {(m.x, m.y) for m in self.mothers}

        mother_receive = self.foods + self.mothers + self.children + self.threats

        for mother in self.mothers:
            food_perceived, agents_perceived = mother.scan_perception(
                mother_receive, perception_range=perception_range
            )

            # choose goal
            goal = None
            target_food = None

            if food_perceived:
                food_perceived.sort(key=lambda t: t[1])
                target_food = food_perceived[0][0]
                goal = (target_food.x, target_food.y)
            else:
                child = mother.child
            #     if child is not None and not child.is_carried:
            #         goal = (child.x, child.y)

            #     else :
            #         proposals[mother] =(mother.x, mother.y)
            #         child.x, child.y = mother.x, mother.y
                    
            #         continue

                if child is None:
                    proposals[mother] = (mother.x, mother.y)
                    continue
                if child.is_carried:
                    proposals[mother] = (mother.x, mother.y)
                    child.x, child.y = mother.x, mother.y
                    continue
                goal = (child.x, child.y)

            if goal is None:
                # nx, ny = mother.x, mother.y
                continue

            # dynamic obstacles: other mothers (not self)
            blocked = {(m.x, m.y) for m in self.mothers if m is not mother}

            path = astar(
                start=(mother.x, mother.y),
                goal=goal,
                grid_w=self.grid_w,
                grid_h=self.grid_h,
                blocked=blocked,
                moves_8=True
            )
            print(path)
            if path is None or len(path) < 2:
                # fallback: your local best_step
                nx, ny = self.best_step(mother, goal, occupied_now)
            else:
                nx, ny = path[1]  # next step

                # if next step is occupied RIGHT NOW (rare due to blocked), fallback
                if (nx, ny) in occupied_now and (nx, ny) != (mother.x, mother.y):
                    nx, ny = self.best_step(mother, goal, occupied_now)

            proposals[mother] = (nx, ny)

            # if reached child -> carry
            child = mother.child
            if child is not None and (nx, ny) == (child.x, child.y) and not food_perceived:
                child.set_carried(True)
                # child.x, child.y = nx, ny

            # if reached food -> collect
            if target_food is not None and (nx, ny) == (target_food.x, target_food.y) and not target_food.collected:
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
        avoid_range = 1
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


import heapq
import math

def octile_heuristic(a, b):
    # good for 8-neighbor grids
    (x1, y1), (x2, y2) = a, b
    dx, dy = abs(x1 - x2), abs(y1 - y2)
    # diagonal cost ~ sqrt(2), straight cost 1
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

def astar(start, goal, grid_w, grid_h, blocked, moves_8=True):
    if start == goal:
        return [start]

    if moves_8:
        nbrs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        def step_cost(dx, dy):  # diagonal costs more
            return math.sqrt(2) if dx != 0 and dy != 0 else 1.0
        h = octile_heuristic
    else:
        nbrs = [(1,0),(-1,0),(0,1),(0,-1)]
        def step_cost(dx, dy):
            return 1.0
        h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_heap = []
    heapq.heappush(open_heap, (h(start, goal), 0.0, start))
    came_from = {}
    gscore = {start: 0.0}
    closed = set()

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)

        if cur == goal:
            # reconstruct
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path

        cx, cy = cur
        for dx, dy in nbrs:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < grid_w and 0 <= ny < grid_h):
                continue

            nxt = (nx, ny)

            # blocked cells are not allowed EXCEPT goal
            if nxt in blocked and nxt != goal:
                continue

            ng = g + step_cost(dx, dy)
            if ng < gscore.get(nxt, float("inf")):
                gscore[nxt] = ng
                came_from[nxt] = cur
                f = ng + h(nxt, goal)
                heapq.heappush(open_heap, (f, ng, nxt))

    return None  # no path
