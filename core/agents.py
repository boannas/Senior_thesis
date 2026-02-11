import math
from core.entities import Food


class Agent:
    """Base class for agents (mother and child)"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, agent_type="mother"):
        # Initialize agent position and type
        self.x, self.y = x, y 
        self.grid_w = grid_w 
        self.grid_h = grid_h 
        self.agent_type = agent_type  # "mother" or "child"

        # --- Survival & Resources ---
        self.hp = float(hp)          # survival
        self.energy = float(energy)    # resource for actions
        self.age = 0.0              # agent age

        # --- Physiological states ---
        self.fatigue = 0.0
        self.injury = 0.0
        self.stress = 0.0

        # --- Hormone ---
        self.OT = 0.0  # Oxytocin
        self.CT = 0.0  # Cortisol

        # --- Psychological states ---
        self.bond_strength = 0.0
        self.separation_distress = 0.0
        self.risk_estimate = 0.0
        self.attention_focus = "NONE"

    
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
            if dist > perception_range: # Out of perception range
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
        self.holding_food = False
        # self.motivations = ['Protect', 'Care', 'Forage', 'Self']
        self.current_motivation = ""

    def set_child(self, child_agent):
        """Set the child agent being carried by mother"""
        self.child = child_agent

    def picking_food(self, picking: bool):
        self.holding_food = picking

    def update_physiology(self, moved=False, acted=False):
        # Costs 
        if moved:
            self.energy -= 0.2
            self.fatigue -= 0.05
        if acted:
            self.energy -= 0.1
            self.fatigue -= 0.02

        self.energy = max(0.0, self.energy)
        self.fatigue = min(100.0, self.fatigue)

        if self.energy <= 0.0:
            self.hp -= 0.1

        # print(self.energy, self.fatigue)

    
    def compute_scores(self, child, threats, foods):
        E_ideal = 100.0
        energy_def = max(0.0, (E_ideal - self.energy) / E_ideal)   # 0..1

        # child need (recommend: use child.hunger; if not, use child.distress or energy proxy)
        child_need = 0.0
        if child is not None and child.is_alive() and (not child.is_carried):
            # If you don't have hunger yet, use distress; otherwise use hunger.
            # child_need = min(1.0, child.hunger / 100.0)
            child_need = min(1.0, child.distress / 100.0)

        # threat level (for now, you can keep 0 if you haven't implemented threat behavior)
        threat_level = 0.0
        for t in threats:
            d = abs(t[0].x - self.x) + abs(t[0].y - self.y)
            if d > 0:
                threat_level = max(threat_level, 1.0 / d)

        # self risk
        fatigue = 0.0 if self.fatigue is None else self.fatigue
        injury  = 0.0 if self.injury is None else self.injury
        self_risk = min(1.0, (fatigue / 100.0) + (injury / 100.0))

        # scores (tune weights later)
        M_care   = 2.0 * child_need + 1.0 * (self.bond_strength or 0.0) - 0.5 * energy_def
        M_forage = 2.0 * energy_def - 1.5 * threat_level
        M_self   = 3.0 * self_risk + 1.0 * energy_def
        M_prot   = 3.0 * threat_level + 1.0 * (self.bond_strength or 0.0) - 2.0 * self_risk

        return {"CARE": M_care, "FORAGE": M_forage, "SELF": M_self, "PROTECT": M_prot}

    def select_motivation(self, scores, margin=0.15):
        best = max(scores, key=scores.get)
        cur = getattr(self, "current_motivation", None)
        if cur in scores and scores[best] < scores[cur] + margin:
            best = cur
        self.current_motivation = best
        return best
    
class ChildAgent(Agent):
    """Child agent - can be carried by mother"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, child_id=None):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="child")
        self.id = child_id
        self.mother = None  # Reference to mother agent
        self.is_carried = False

        self.distress = 0.0
        self.hunger = 0.0

    def set_mother(self, mother_agent):
        """Set the mother agent carrying this child"""
        self.mother = mother_agent

    def set_carried(self, carried: bool):
        """Set whether the child is being carried by mother"""
        self.is_carried = carried

    def update(self):
        self.hunger = min(100.0, self.hunger + 0.2)
        self.distress = min(100.0, self.distress + 0.05)

class ThreatAgent(Agent):
    """Threat agent - can move and pose danger to mother and child"""
    def __init__(self, x, y, grid_w, grid_h, hp, energy, threat_id=None):
        super().__init__(x, y, grid_w, grid_h, hp, energy, agent_type="threat")
        self.id = threat_id

