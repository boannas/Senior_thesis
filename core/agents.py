"""
Agent classes for the maternal instinct simulation.

Agents:
- Agent        : Base class with position, energy, perception, movement.
- MotherAgent  : Autonomous caregiver with hormonal/psychological states and motivations.
- ChildAgent   : Vulnerable dependent with hunger, warmth, injury needs.
- ThreatAgent  : Predator with patrol/chase/flee behavior.
"""

import math
import copy
from core.entities import Food
from core.policies.deficit import IDEAL_VALUE, deficit_low
from core.seed import rng


# ─── Movement / Action Costs ───────────────────────────────────────────

MOVE_COST = {
    "energy": -0.3,     # Reduced from -1.0 to make care-strategy sustainable
    "fatigue": 0.5,
}

ACTION_COST = {
    "energy": -0.1,
    "fatigue": 0.1,
}

REST_RECOVERY = {
    "energy": 0.1,      # Changed from -0.05: mother recovers energy when resting
    "fatigue": -3.0,    # Active rest recovers fatigue fast
}

IDLE_RECOVERY = {
    "energy": 0.05,     # Changed from -0.05: mother recovers energy during idle (sustainability)
    "fatigue": -1.0,    # Fatigue decreases when idle
}


# ─── Helper ─────────────────────────────────────────────────────────────

def weighted_sum(weight_group, keys):
    """Sum the weights for the given keys in a weight group dict."""
    return sum(weight_group[k] for k in keys)


# ═══════════════════════════════════════════════════════════════════════
# Base Agent
# ═══════════════════════════════════════════════════════════════════════

class Agent:
    """Base class for all agents (mother, child, threat)."""

    def __init__(self, x, y, grid_width, grid_height, energy, agent_type="mother"):
        self.x = x
        self.y = y
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.agent_type = agent_type

        self.age = 0.0
        self.energy = energy
        self.alive = True

    def is_alive(self):
        """Check if agent is still alive (alive flag and energy > 0)."""
        return self.alive and self.energy >= 0.0

    def distance_to(self, other_x, other_y, metric="octile"):
        """
        Compute distance from this agent to (other_x, other_y).

        Supported metrics: 'manhattan', 'chebyshev', 'euclidean', 'octile'.
        """
        dx = abs(self.x - other_x)
        dy = abs(self.y - other_y)

        if metric == "manhattan":
            return dx + dy
        elif metric == "chebyshev":
            return max(dx, dy)
        elif metric == "euclidean":
            return math.hypot(dx, dy)
        elif metric == "octile":
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
        else:
            raise ValueError(f"Unknown distance metric: {metric}")

    def scan_perception(self, entities, perception_range):
        """
        Scan for entities within perception range.

        Returns
        -------
        food_list : list of (Food, distance)
            Visible, uncollected food items.
        agents_by_type : list of [mothers, children, threats]
            Each is a list of (Agent, distance) tuples.
        """
        food_list = []
        mothers_seen = []
        children_seen = []
        threats_seen = []

        for entity in entities:
            if entity is self:
                continue

            dist = self.distance_to(entity.x, entity.y, metric="octile")
            if dist > perception_range:
                continue

            if isinstance(entity, Food):
                if not getattr(entity, "collected", False):
                    food_list.append((entity, dist))
            elif isinstance(entity, MotherAgent):
                mothers_seen.append((entity, dist))
            elif isinstance(entity, ChildAgent) and entity.is_alive():
                children_seen.append((entity, dist))
            elif isinstance(entity, ThreatAgent):
                threats_seen.append((entity, dist))

        agents_by_type = [mothers_seen, children_seen, threats_seen]
        return food_list, agents_by_type

    def step_towards(self, target_x, target_y):
        """Return (dx, dy) direction to step toward target. Returns (0,0) if dead."""
        if not self.is_alive():
            return (0, 0)

        dx = 0 if target_x == self.x else (1 if target_x > self.x else -1)
        dy = 0 if target_y == self.y else (1 if target_y > self.y else -1)
        return (dx, dy)


# ═══════════════════════════════════════════════════════════════════════
# Mother Agent
# ═══════════════════════════════════════════════════════════════════════

class MotherAgent(Agent):
    """
    Mother agent — autonomous caregiver with:
    - Physiological states: energy, fatigue
    - Hormones: oxytocin (OT), cortisol (CORT)
    - Psychological states: bonding, fear, stress, closeness_to_child
    - Motivation system: Forage / Care / Self / Protect
    - Genetic (fixed) + learned (plastic) weight system
    """

    def __init__(self, x, y, grid_width, grid_height, energy, mother_id=None, use_fixed_weights=False, baseline_weights=None):
        super().__init__(x, y, grid_width, grid_height, energy, agent_type="mother")
        self.id = mother_id
        self.child = None               # Reference to assigned child agent
        self.holding_food = False
        self.holding_child = False
        self.food_inventory = 0

        # --- Physiological states ---
        self.energy = energy
        self.fatigue = 0.0

        # --- Psychological states ---
        self.bonding = 100.0
        self.fear_threat = 0.0
        self.stress = 0.0
        self.closeness_child = 0.0

        # --- Hormones ---
        self.oxytocin = 0.0             # Bonding hormone (proximity to child)
        self.cortisol = 0.0             # Stress hormone (threats, needs, energy deficit)

        # --- Motivation scores ---
        self.motivations = {
            "Forage":   0.0,
            "Care":     0.0,
            "Self":     0.0,
            "Protect":  0.0,
        }
        self.selected_motivation = "Self"

        # --- Genetic weights (fixed, inherited, never change) ---
        # Priority: baseline_weights (from BASELINE_0_WEIGHT_CONFIG) > use_fixed_weights flag > default randomization
        if baseline_weights is not None:
            # BASELINE-CUSTOM: Use provided weights from systematic search
            self.psych_weights_fixed = {
                "child_need":   {"hunger": 0.5, "warmth": 0.5, "injury": 0.5},
                "oxytocin":     {"closeness_gain": 0.5, "decay": 0.5},
                "bonding":      {"oxytocin_gain": 0.5, "child_need_decay": 0.5, "child_absent_decay": 0.5},
                "cortisol":     {"threat_gain": 0.5, "child_need_gain": 0.5, "energy_deficit_gain": 0.5, "decay": 0.5},
                "stress":       {"cortisol_gain": 0.5, "fear_gain": 0.5, "child_need_gain": 0.5, "decay": 0.5},
                "fear":         {"threat_gain": 0.5, "decay": 0.5},
            }
        elif use_fixed_weights:
            # BASELINE-0 (Controlled): All weights = 0.5 for neutrality
            self.psych_weights_fixed = {
                "child_need":   {"hunger": 0.5, "warmth": 0.5, "injury": 0.5},
                "oxytocin":     {"closeness_gain": 0.5, "decay": 0.5},
                "bonding":      {"oxytocin_gain": 0.5, "child_need_decay": 0.5, "child_absent_decay": 0.5},
                "cortisol":     {"threat_gain": 0.5, "child_need_gain": 0.5, "energy_deficit_gain": 0.5, "decay": 0.5},
                "stress":       {"cortisol_gain": 0.5, "fear_gain": 0.5, "child_need_gain": 0.5, "decay": 0.5},
                "fear":         {"threat_gain": 0.5, "decay": 0.5},
            }
        else:
            # BASELINE-1 (Heterogeneous): Randomized U(0,1) weights for biological diversity
            self.psych_weights_fixed = {
                "child_need":   {"hunger": rng.uniform(0, 1), "warmth": rng.uniform(0, 1), "injury": rng.uniform(0, 1)},
                "oxytocin":     {"closeness_gain": rng.uniform(0, 1), "decay": rng.uniform(0, 1)},
                "bonding":      {"oxytocin_gain": rng.uniform(0, 1), "child_need_decay": rng.uniform(0, 1), "child_absent_decay": rng.uniform(0, 1)},
                "cortisol":     {"threat_gain": rng.uniform(0, 1), "child_need_gain": rng.uniform(0, 1), "energy_deficit_gain": rng.uniform(0, 1), "decay": rng.uniform(0, 1)},
                "stress":       {"cortisol_gain": rng.uniform(0, 1), "fear_gain": rng.uniform(0, 1), "child_need_gain": rng.uniform(0, 1), "decay": rng.uniform(0, 1)},
                "fear":         {"threat_gain": rng.uniform(0, 1), "decay": rng.uniform(0, 1)},
            }

        # --- Motivation weights (fixed, inherited) ---
        if baseline_weights is not None:
            # BASELINE-CUSTOM: Use provided weights from systematic search
            self.motivation_weights_fixed = baseline_weights
        elif use_fixed_weights:
            # BASELINE-0 (Controlled): All weights = 0.5
            self.motivation_weights_fixed = {
                "forage":   {"child_hunger": 0.5, "energy_deficit": 0.5, "low_fear": 0.5},
                "care":     {"child_warmth": 0.5, "closeness_deficit": 0.5, "bonding": 0.5},
                "self":     {"fatigue": 0.5, "fear": 0.5, "stress": 0.5},
                "protect":  {"child_injury": 0.5, "fear": 0.5, "closeness_deficit": 0.5, "bonding": 0.5},
            }
        else:
            # BASELINE-1 (Heterogeneous): Randomized U(0,1) weights
            self.motivation_weights_fixed = {
                "forage":   {"child_hunger": rng.uniform(0, 1), "energy_deficit": rng.uniform(0, 1), "low_fear": rng.uniform(0, 1)},
                "care":     {"child_warmth": rng.uniform(0, 1), "closeness_deficit": rng.uniform(0, 1), "bonding": rng.uniform(0, 1)},
                "self":     {"fatigue": rng.uniform(0, 1), "fear": rng.uniform(0, 1), "stress": rng.uniform(0, 1)},
                "protect":  {"child_injury": rng.uniform(0, 1), "fear": rng.uniform(0, 1), "closeness_deficit": rng.uniform(0, 1), "bonding": rng.uniform(0, 1)},
            }

        # --- Plastic weights (learned, adapt via outcome-gated plasticity) ---
        self.psych_weights_plastic = copy.deepcopy(self.psych_weights_fixed)
        self.motivation_weights_plastic = copy.deepcopy(self.motivation_weights_fixed)
        self.learning_rate = 0.02

    @property
    def psych_weights(self):
        """Effective psych weights = average of fixed + plastic."""
        return {
            category: {
                key: (self.psych_weights_fixed[category][key] + self.psych_weights_plastic[category][key]) / 2.0
                for key in self.psych_weights_fixed[category]
            }
            for category in self.psych_weights_fixed
        }

    @property
    def motivation_weights(self):
        """Effective motivation weights = average of fixed + plastic."""
        return {
            category: {
                key: (self.motivation_weights_fixed[category][key] + self.motivation_weights_plastic[category][key]) / 2.0
                for key in self.motivation_weights_fixed[category]
            }
            for category in self.motivation_weights_fixed
        }

    def set_child(self, child_agent):
        """Assign a child agent to this mother."""
        self.child = child_agent

    def picking_food(self, is_picking):
        """Set whether mother is holding food."""
        self.holding_food = is_picking

    def picking_child(self, is_picking):
        """Set whether mother is holding child."""
        self.holding_child = is_picking

    def update_physiology(self, moved=False, acted=False, rested=False):
        """
        Update energy and fatigue based on what the mother did this tick.
        - moved: spent energy moving
        - acted: spent energy performing an action
        - rested: actively resting (fast fatigue recovery)
        - none of the above: idle recovery
        """
        if not self.is_alive():
            self.alive = False
            return

        if moved:
            self.energy += MOVE_COST["energy"]
            self.fatigue += MOVE_COST["fatigue"]
        elif acted:
            self.energy += ACTION_COST["energy"]
            self.fatigue += ACTION_COST["fatigue"]
        elif rested:
            # Active rest: fast fatigue recovery
            self.energy = max(0.0, self.energy + REST_RECOVERY["energy"])
            self.fatigue = max(0.0, self.fatigue + REST_RECOVERY["fatigue"])
        else:
            # Idle: moderate recovery
            self.energy = max(0.0, self.energy + IDLE_RECOVERY["energy"])
            self.fatigue = max(0.0, self.fatigue + IDLE_RECOVERY["fatigue"])

        # Clamp to [0, 100]
        self.energy = max(0.0, min(100.0, self.energy))
        self.fatigue = max(0.0, min(100.0, self.fatigue))

        if self.energy <= 0.0:
            self.alive = False

    def update_psychology_and_hormones(self, world):
        """
        Update all psychological and hormonal states based on the
        current world state (child needs, threat proximity, etc.).
        """
        if not self.is_alive():
            self.alive = False
            return

        pw = self.psych_weights  # shorthand for effective psych weights

        # --- Baseline values ---
        child_need = 0.0
        energy_deficit = deficit_low(self.energy, IDEAL_VALUE['M_energy'])

        fear_increase = 0.0
        fear_decay = pw["fear"]["decay"] * self.fear_threat

        oxytocin_increase = 0.0
        oxytocin_decay = pw["oxytocin"]["decay"] * self.oxytocin

        cortisol_increase = 0.0
        cortisol_decay = pw["cortisol"]["decay"] * self.cortisol

        closeness_to_child = 0.0
        stress_decay = pw["stress"]["decay"] * self.stress

        # --- Child-dependent calculations ---
        if self.child is not None and self.child.is_alive():
            child = self.child

            # Closeness (inverse of normalized distance)
            distance_to_child = self.distance_to(child.x, child.y, metric='octile')
            normalized_dist = min(1.0, distance_to_child / max(world.grid_w, world.grid_h))
            closeness_to_child = 100.0 * (1.0 - normalized_dist)

            # Child need signals (normalized to [0, 1])
            hunger_signal = child.hunger / 100.0
            warmth_signal = abs(child.warmth - 50.0) / 50.0
            injury_signal = child.injury / 100.0

            child_need_weight_sum = weighted_sum(
                pw["child_need"], ["hunger", "warmth", "injury"]
            )
            child_need = 100.0 * (
                pw["child_need"]["hunger"] * hunger_signal +
                pw["child_need"]["warmth"] * warmth_signal +
                pw["child_need"]["injury"] * injury_signal
            ) / child_need_weight_sum

            bonding_decay = pw["bonding"]["child_need_decay"] * child_need

            # Oxytocin: increases when very close to child (≥90% closeness)
            if closeness_to_child >= 90:
                oxytocin_increase = pw["oxytocin"]["closeness_gain"] * closeness_to_child

            # Fear: driven by threat proximity to child
            threat_proximity = 0.0
            threat_radius = 10.0
            for threat in world.threats:
                dist = child.distance_to(threat.x, threat.y, metric='octile')
                if dist <= threat_radius:
                    threat_proximity = max(threat_proximity, (threat_radius - dist) / threat_radius)

            fear_increase = 100.0 * (pw["fear"]["threat_gain"] * threat_proximity)

            # Cortisol: driven by threats, child needs, and energy deficit
            cortisol_gain_sum = weighted_sum(
                pw["cortisol"], ["threat_gain", "child_need_gain", "energy_deficit_gain"]
            )
            cortisol_increase = 100.0 * (
                pw["cortisol"]["threat_gain"] * threat_proximity +
                pw["cortisol"]["child_need_gain"] * (child_need / 100.0) +
                pw["cortisol"]["energy_deficit_gain"] * (energy_deficit / 100.0)
            ) / cortisol_gain_sum

        # --- Child dead or absent ---
        else:
            closeness_to_child = 0.0
            bonding_decay = pw["bonding"]["child_absent_decay"] * self.bonding

            cortisol_gain_sum = weighted_sum(
                pw["cortisol"], ["threat_gain", "child_need_gain", "energy_deficit_gain"]
            )
            cortisol_increase = 100.0 * (
                pw["cortisol"]["energy_deficit_gain"] * (energy_deficit / 100.0)
            ) / cortisol_gain_sum

        # --- Stress (composite) ---
        stress_gain_sum = weighted_sum(
            pw["stress"], ["cortisol_gain", "fear_gain", "child_need_gain"]
        )
        stress_increase = (
            pw["stress"]["cortisol_gain"] * self.cortisol +
            pw["stress"]["fear_gain"] * self.fear_threat +
            pw["stress"]["child_need_gain"] * child_need
        ) / stress_gain_sum

        # --- Apply updates with clamping to [0, 100] ---
        self.closeness_child = max(0.0, min(100.0, closeness_to_child))
        self.oxytocin = max(0.0, min(100.0, self.oxytocin + oxytocin_increase - oxytocin_decay))
        self.bonding = max(0.0, min(100.0,
            self.bonding + pw["bonding"]["oxytocin_gain"] * self.oxytocin - bonding_decay
        ))
        self.fear_threat = max(0.0, min(100.0, self.fear_threat + fear_increase - fear_decay))
        self.cortisol = max(0.0, min(100.0, self.cortisol + cortisol_increase - cortisol_decay))
        self.stress = max(0.0, min(100.0, self.stress + stress_increase - stress_decay))

    def print_state(self):
        """Print current mother state for debugging."""
        print(f"\n[MOTHER {self.id}]")
        print(f"  Energy: {self.energy:.1f}  Fatigue: {self.fatigue:.1f}")
        print(f"  Bonding: {self.bonding:.1f}  Fear: {self.fear_threat:.1f}  Stress: {self.stress:.1f}")
        print(f"  Closeness to Child: {self.closeness_child:.1f}")
        print(f"  Oxytocin: {self.oxytocin:.1f}  Cortisol: {self.cortisol:.1f}")
        if self.child:
            print(f"  Child: {self.child.id} (alive={self.child.alive})")
        print(f"  Alive: {self.alive}")


# ═══════════════════════════════════════════════════════════════════════
# Child Agent
# ═══════════════════════════════════════════════════════════════════════

class ChildAgent(Agent):
    """
    Child agent — passive/vulnerable entity with:
    - Hunger: increases each tick, reduced by being fed
    - Warmth: decreases when not carried, restored by mother's care
    - Injury: caused by nearby threats, heals when threats are far
    """

    def __init__(self, x, y, grid_width, grid_height, energy, child_id=None):
        super().__init__(x, y, grid_width, grid_height, energy, agent_type="child")
        self.id = child_id
        self.mother = None
        self.is_carried = False

        # Internal needs (0 = ideal, 100 = critical)
        self.hunger = 0.0       # 0: full → 100: starving
        self.warmth = 50.0      # 0: freezing → 50: ideal → 100: overheating
        self.injury = 0.0       # 0: healthy → 100: lethal

        self.threat_recovery_radius = 5.0
        self.injury_recovery_rate = 0.1
        self.warmth_recovery_rate = 0.1

        self.threat_close = False
        self.nearest_threat_dist = float("inf")

    def is_alive(self):
        """Child dies if hunger/injury reach 100, or warmth hits 0 or 100."""
        if not self.alive:
            return False
        if self.hunger >= 100:
            return False
        if self.warmth <= 0 or self.warmth >= 100:
            return False
        if self.injury >= 100:
            return False
        return True

    def set_mother(self, mother_agent):
        """Assign a mother agent to this child."""
        self.mother = mother_agent

    def set_carried(self, carried):
        """Set whether child is being carried by mother."""
        self.is_carried = carried

    def update(self, world):
        """
        Update child's internal states each tick:
        - Hunger always increases
        - Warmth recovers when carried, drops when alone
        - Injury recovers when no threats are nearby
        """
        if not self.is_alive():
            self.alive = False
            return

        self.threat_close = False
        self.nearest_threat_dist = float("inf")

        # Hunger increases each tick (reduced from 1.0 for survivability)
        self.hunger = min(100.0, self.hunger + 0.5)

        # Warmth: carried = warm up, alone = cool down slowly
        if self.is_carried:
            self.warmth = min(50.0, self.warmth + self.warmth_recovery_rate)
        else:
            self.warmth = max(0.0, self.warmth - 0.2)  # Was 0.5, reduced for survivability

        # Injury: check threat proximity
        for threat in world.threats:
            dist = self.distance_to(threat.x, threat.y, metric="octile")
            self.nearest_threat_dist = min(self.nearest_threat_dist, dist)
            if dist <= self.threat_recovery_radius:
                self.threat_close = True

        # Recover from injury only when threats are far away
        if not self.threat_close:
            self.injury = max(0.0, self.injury - self.injury_recovery_rate)

        if not self.is_alive():
            self.alive = False

    def print_state(self):
        """Print current child state for debugging."""
        print(f"\n[CHILD {self.id}]")
        print(f"  Hunger: {self.hunger:.1f}  Warmth: {self.warmth:.1f}  Injury: {self.injury:.1f}")
        print(f"  Carried: {self.is_carried}")
        if self.mother:
            print(f"  Mother: {self.mother.id}")
        print(f"  Alive: {self.alive}")


# ═══════════════════════════════════════════════════════════════════════
# Threat Agent
# ═══════════════════════════════════════════════════════════════════════

class ThreatAgent(Agent):
    """
    Threat agent — predator that patrols, chases children,
    and flees from mothers.
    """

    def __init__(self, x, y, grid_width, grid_height, energy, threat_id=None):
        super().__init__(x, y, grid_width, grid_height, energy, agent_type="threat")
        self.id = threat_id
        self.patrol_goal = None
        self.patrol_timer = 0
        self.perception_range = 2
        self.energy = 100.0

        self.mode = "patrol"            # "patrol", "chase", or "flee"
        self.flee_timer = 0
        self.last_seen_mothers = []
        self.last_seen_memory_timer = 0  # ticks since last saw mother; clears memory after timeout

    def print_state(self):
        """Print current threat state for debugging."""
        print(f"\n[THREAT {self.id}]")
        print(f"  Position: ({self.x}, {self.y})  Mode: {self.mode}")
        print(f"  Energy: {self.energy:.1f}  Patrol goal: {self.patrol_goal}")
        print(f"  Alive: {self.alive}")