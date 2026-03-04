import random
from core.agents import ChildAgent, MotherAgent, ThreatAgent
from core.entities import Food
from core.policies.mother import mother_policy_propose, apply_mother_intents
from core.policies.threat import threat_policy_propose, apply_threat_intents
from core.sim.movement import resolve_and_apply_moves
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
            self.mothers.append(MotherAgent(mx, my, grid_w, grid_h, energy=100.0, mother_id=f"M{i}"))

        # Create child agents
        for i, (cx, cy) in enumerate(child_start):
            self.children.append(ChildAgent(cx, cy, grid_w, grid_h, energy=10.0, child_id=f"C{i}"))

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
            self.threats.append(ThreatAgent(cx, cy, grid_w, grid_h, energy=50))

    def step(self, dt: float = 0.1):

        # Internal dynamics before decision 
        for c in self.children:
            # print(c.id)
            pass
            # c.update(self)
        

        for m in self.mothers:
            m.update_psych_neuro(self)
            # print(m.id)
        
        m_prop, m_int = mother_policy_propose(self)
        t_prop, t_int = threat_policy_propose(self)

        all_agents = self.mothers + self.threats
        all_prop = {**m_prop, **t_prop}

        resolve_and_apply_moves(all_agents, all_prop, self.grid_w, self.grid_h)

        apply_mother_intents(self, m_int)
        apply_threat_intents(self, t_int)

        self.t += dt
        