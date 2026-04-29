"""
World: the simulation environment containing agents, food, and threats.
Manages the main simulation loop (step), entity spawning, cleanup,
and history recording for plotting.
"""

import random
from core.agents import ChildAgent, MotherAgent, ThreatAgent
from core.entities import Food
from core.policies.mother import mother_policy_propose, apply_mother_intents
from core.policies.threat import threat_policy_propose, apply_threat_intents
from core.sim.movement import resolve_and_apply_moves
from collections import defaultdict, deque


class World:
    """
    Grid-world simulation environment.

    Each tick: update child/mother states → compute motivations →
    plan movement → resolve collisions → apply actions → cleanup → record.
    """

    def __init__(
        self,
        grid_w,
        grid_h,
        mother_starts=None,
        child_start=None,
        food_positions=None,
        threat_starts=None,
        seed=42,
        day_step=None,
        plasticity_rule="outcome",
        plasticity_deficit_signal="global",
        plasticity_learn_w=True,
        plasticity_update_mode="per_tick",
        plasticity_segment_kmax=20,
        food_spawn_interval=None,
        food_spawn_n=1,
        use_fixed_weights=False,
        baseline_weights=None,
    ):
        random.seed(seed)

        self.tick = 0
        self.t = 0.0
        self.grid_w = grid_w
        self.grid_h = grid_h
        self.day_step = day_step
        self.plasticity_rule = plasticity_rule
        # Plasticity configuration:
        # - deficit signal: "global" (overall deficit) or "local" (motivation-aligned deficit)
        # - learn_w: whether to update psych/state dynamics weights (w); if False, only u updates
        self.plasticity_deficit_signal = plasticity_deficit_signal
        self.plasticity_learn_w = bool(plasticity_learn_w)
        # - update_mode:
        #   * "per_tick": current behavior (compare deficit_before vs deficit_after each tick)
        #   * "segment": update only when motivation changes (credit assignment over a run)
        #   * "segment_capped": like segment, but also update every K ticks if the run persists
        self.plasticity_update_mode = plasticity_update_mode
        self.plasticity_segment_kmax = int(plasticity_segment_kmax)
        self.food_spawn_interval = food_spawn_interval
        self.food_spawn_n = food_spawn_n

        # 8-directional movement + stay
        self.RANDOM_MOVES = [
            (0, 0), (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ]
        self.PATROL_TIMEOUT = (self.grid_w * self.grid_h) // 2

        # --- Create agents ---
        self.mothers = []
        self.children = []
        self.threats = []

        for i, (mx, my) in enumerate(mother_starts):
            self.mothers.append(MotherAgent(mx, my, grid_w, grid_h, energy=100.0, mother_id=f"M{i}", use_fixed_weights=use_fixed_weights, baseline_weights=baseline_weights))

        for i, (cx, cy) in enumerate(child_start):
            self.children.append(ChildAgent(cx, cy, grid_w, grid_h, energy=10.0, child_id=f"C{i}"))

        # Link mothers ↔ children (1:1 by index)
        for mother, child in zip(self.mothers, self.children):
            mother.set_child(child)
            child.set_mother(mother)

        # --- Create food ---
        self.foods = []
        if food_positions:
            for (fx, fy) in food_positions:
                self.foods.append(Food(fx, fy))

        # --- Create threats ---
        for i, (tx, ty) in enumerate(threat_starts):
            self.threats.append(ThreatAgent(tx, ty, grid_w, grid_h, energy=50))

        # --- History for plotting ---
        self.history_len = 300
        self.tick_history = deque(maxlen=self.history_len)

        self.mother_history = defaultdict(lambda: {
            # Physiological
            "energy": deque(maxlen=self.history_len),
            "fatigue": deque(maxlen=self.history_len),
            # Psychological
            "bonding": deque(maxlen=self.history_len),
            "fear_threat": deque(maxlen=self.history_len),
            "stress": deque(maxlen=self.history_len),
            "closeness_child": deque(maxlen=self.history_len),
            # Hormones
            "oxytocin": deque(maxlen=self.history_len),
            "cortisol": deque(maxlen=self.history_len),
            # Motivation values
            "mot_forage": deque(maxlen=self.history_len),
            "mot_care": deque(maxlen=self.history_len),
            "mot_self": deque(maxlen=self.history_len),
            "mot_protect": deque(maxlen=self.history_len),
            # Motivation selection (binary)
            "sel_forage": deque(maxlen=self.history_len),
            "sel_care": deque(maxlen=self.history_len),
            "sel_self": deque(maxlen=self.history_len),
            "sel_protect": deque(maxlen=self.history_len),
        })

        self.child_history = defaultdict(lambda: {
            "hunger": deque(maxlen=self.history_len),
            "warmth": deque(maxlen=self.history_len),
            "injury": deque(maxlen=self.history_len),
        })

    # ─── Main Simulation Step ───────────────────────────────────────

    def step(self, dt=0.1):
        """Advance the simulation by one tick."""

        # Check grid has room for spawning
        has_empty, empty_count, occupied = self.check_free_cell
        if not has_empty:
            return

        # 1. Update internal states
        for child in self.children:
            child.update(self)

        for mother in self.mothers:
            mother.update_psychology_and_hormones(self)

        # 2. Compute motivations and propose movements
        mother_proposals, mother_intents = mother_policy_propose(self)
        threat_proposals, threat_intents = threat_policy_propose(self)

        all_agents = self.mothers + self.threats
        all_proposals = {**mother_proposals, **threat_proposals}

        # 3. Resolve collisions and apply moves
        resolve_and_apply_moves(all_agents, all_proposals, self.grid_w, self.grid_h)
        apply_mother_intents(self, mother_intents)
        apply_threat_intents(self, threat_intents)

        # 4. Cleanup
        self._cleanup_dead_agents()
        self._cleanup_collected_food()

        # 5. Periodic food spawning
        spawn_interval = (
            self.food_spawn_interval
            if self.food_spawn_interval is not None
            else max(1, self.day_step // 10)
        )
        if self.tick % spawn_interval == 0 and self.tick != 0:
            self._spawn_random_food(count=self.food_spawn_n, empty_count=empty_count, occupied=occupied)

        # 6. Record history
        self._record_child_states()
        self._record_mother_states()

        self.t += dt
        self.tick += 1

    # ─── Time Properties ────────────────────────────────────────────

    @property
    def time_of_day(self):
        """Current tick within the day cycle."""
        return self.tick % self.day_step

    @property
    def is_day(self):
        """True if it's daytime (first half of the day cycle)."""
        return self.time_of_day < self.day_step // 2

    @property
    def day_count(self):
        """Current day number (1-indexed)."""
        return self.tick // self.day_step + 1

    @property
    def has_living_agents(self):
        """True if any mothers or children are still alive."""
        return len(self.mothers) > 0 or len(self.children) > 0

    @property
    def check_free_cell(self):
        """
        Check if there are free cells on the grid.

        Returns
        -------
        has_empty : bool
        empty_count : int or None
        occupied : set or None
        """
        occupied = {(m.x, m.y) for m in self.mothers}
        occupied |= {(c.x, c.y) for c in self.children}
        occupied |= {(f.x, f.y) for f in self.foods}

        total_cells = self.grid_w * self.grid_h
        empty_count = total_cells - len(occupied)

        if empty_count <= 0:
            print('[ERROR]: Grid is full — cannot spawn new entities')
            return False, None, None
        return True, empty_count, occupied

    # ─── Internal Methods ───────────────────────────────────────────

    def _spawn_random_food(self, count=1, empty_count=0, occupied=None):
        """Spawn food at random unoccupied cells."""
        count = min(count, empty_count)
        for _ in range(count):
            while True:
                x = random.randint(0, self.grid_w - 1)
                y = random.randint(0, self.grid_h - 1)
                if (x, y) not in occupied:
                    self.foods.append(Food(x, y))
                    occupied.add((x, y))
                    break

    def _cleanup_dead_agents(self):
        """Remove dead agents and unlink their references."""
        for child in [c for c in self.children if not c.is_alive()]:
            if child.mother is not None:
                child.mother.child = None

        for mother in [m for m in self.mothers if not m.is_alive()]:
            if mother.child is not None:
                mother.child.mother = None

        self.children = [c for c in self.children if c.is_alive()]
        self.mothers = [m for m in self.mothers if m.is_alive()]

    def _cleanup_collected_food(self):
        """Remove collected food from the world."""
        self.foods = [f for f in self.foods if not f.collected]

    def _record_mother_states(self):
        """Record current mother states into history for plotting."""
        self.tick_history.append(self.tick)

        for mother in self.mothers:
            history = self.mother_history[mother.id]

            # Physiological
            history["energy"].append(mother.energy)
            history["fatigue"].append(mother.fatigue)

            # Psychological
            history["bonding"].append(mother.bonding)
            history["fear_threat"].append(mother.fear_threat)
            history["stress"].append(mother.stress)
            history["closeness_child"].append(mother.closeness_child)

            # Hormones
            history["oxytocin"].append(mother.oxytocin)
            history["cortisol"].append(mother.cortisol)

            # Motivation values
            history["mot_forage"].append(mother.motivations["Forage"])
            history["mot_care"].append(mother.motivations["Care"])
            history["mot_self"].append(mother.motivations["Self"])
            history["mot_protect"].append(mother.motivations["Protect"])

            # Selected motivation (one-hot)
            selected = max(mother.motivations, key=mother.motivations.get)
            history["sel_forage"].append(1 if selected == "Forage" else 0)
            history["sel_care"].append(1 if selected == "Care" else 0)
            history["sel_self"].append(1 if selected == "Self" else 0)
            history["sel_protect"].append(1 if selected == "Protect" else 0)

            # Fixed vs plastic weight tracking
            self._record_weight_history(mother, history, "motivation_weights", "u")
            self._record_weight_history(mother, history, "psych_weights", "w")

    def _record_weight_history(self, mother, history, weight_attr_prefix, history_prefix):
        """Record fixed vs plastic weights into history deques."""
        fixed_attr = f"{weight_attr_prefix}_fixed"
        plastic_attr = f"{weight_attr_prefix}_plastic"

        if not (hasattr(mother, fixed_attr) and hasattr(mother, plastic_attr)):
            return

        fixed_weights = getattr(mother, fixed_attr)
        plastic_weights = getattr(mother, plastic_attr)

        for category in fixed_weights:
            for key in fixed_weights[category]:
                key_fixed = f"{history_prefix}_fixed_{category}_{key}"
                key_plastic = f"{history_prefix}_plastic_{category}_{key}"

                if key_fixed not in history:
                    history[key_fixed] = deque(maxlen=self.history_len)
                    history[key_plastic] = deque(maxlen=self.history_len)

                history[key_fixed].append(fixed_weights[category][key])
                history[key_plastic].append(plastic_weights[category][key])

    def _record_child_states(self):
        """Record current child states into history for plotting."""
        for child in self.children:
            history = self.child_history[child.id]
            history["hunger"].append(child.hunger)
            history["warmth"].append(child.warmth)
            history["injury"].append(child.injury)