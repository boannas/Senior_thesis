"""
Threat agent decision-making policy.

The threat operates as a finite state machine:
- PATROL: Wander randomly, looking for children.
- CHASE:  Pursue a visible, unprotected child.
- FLEE:   Run away from mothers when detected.

Transitions:
  patrol → chase  (child visible, no mother nearby)
  patrol → flee   (mother spotted)
  chase  → flee   (mother spotted)
  flee   → patrol (safe distance reached, no child visible)
  flee   → chase  (safe distance reached, child still visible)
"""

import random
from func.path_finding import astar
from core.sim.movement import best_step


# ─── Constants ──────────────────────────────────────────────────────────

FLEE_HOLD_TICKS = 3         # Minimum ticks to keep fleeing after losing sight of mother
FLEE_SAFE_DISTANCE = 3.0    # Must be this far from last-seen mothers before stopping flee


# ═══════════════════════════════════════════════════════════════════════
# Proposal Generation
# ═══════════════════════════════════════════════════════════════════════

def threat_policy_propose(world):
    """
    For each threat, determine mode and compute movement proposals.

    Returns
    -------
    proposals : dict {threat: (target_x, target_y)}
    intents   : dict with 'prev_pos' and 'target_child'
    """
    proposals = {}
    occupied_now = {(t.x, t.y) for t in world.threats}
    visible_entities = world.mothers + world.children

    target_child_map = {}
    prev_positions = {threat: (threat.x, threat.y) for threat in world.threats}

    for threat in world.threats:
        _, agents_by_type = threat.scan_perception(
            visible_entities, perception_range=threat.perception_range
        )

        mothers_seen = agents_by_type[0] if len(agents_by_type) > 0 else []
        children_seen = agents_by_type[1] if len(agents_by_type) > 1 else []

        goal = None
        next_x, next_y = threat.x, threat.y  # default: stay

        # ─── Mode Transitions ───────────────────────────────────
        if mothers_seen:
            # Mother detected → flee immediately
            threat.mode = "flee"
            threat.flee_timer = FLEE_HOLD_TICKS
            threat.last_seen_mothers = [(m.x, m.y) for m, _ in mothers_seen]
            threat.patrol_goal = None

        elif threat.mode == "flee":
            # Continue fleeing until timer expires and safely away
            if threat.flee_timer > 0:
                threat.flee_timer -= 1
            else:
                if _is_safe_from_mothers(threat, FLEE_SAFE_DISTANCE):
                    if _has_valid_child_target(children_seen):
                        threat.mode = "chase"
                    else:
                        threat.mode = "patrol"
                        threat.patrol_goal = _pick_safe_patrol_goal(
                            world, threat, threat.last_seen_mothers
                        )
                        threat.patrol_timer = world.PATROL_TIMEOUT

        elif _has_valid_child_target(children_seen):
            threat.mode = "chase"

        else:
            threat.mode = "patrol"

        # ─── Execute Current Mode ───────────────────────────────
        if threat.mode == "flee":
            flee_sources = _mothers_to_positions(mothers_seen)
            if not flee_sources:
                flee_sources = threat.last_seen_mothers
            next_x, next_y = _flee_step(threat, flee_sources, world, occupied_now)

        elif threat.mode == "chase":
            children_seen.sort(key=lambda pair: pair[1])
            child, _ = children_seen[0]

            if child is not None and child.is_alive() and not child.is_carried:
                goal = (child.x, child.y)
                target_child_map[threat] = child
            else:
                threat.mode = "patrol"

        if threat.mode == "patrol":
            if (threat.patrol_goal is None
                    or (threat.x, threat.y) == threat.patrol_goal
                    or threat.patrol_timer <= 0):
                if threat.last_seen_mothers:
                    threat.patrol_goal = _pick_safe_patrol_goal(
                        world, threat, threat.last_seen_mothers
                    )
                else:
                    threat.patrol_goal = _pick_patrol_goal(world, threat)
                threat.patrol_timer = world.PATROL_TIMEOUT

            goal = threat.patrol_goal
            threat.patrol_timer -= 1

        # ─── Pathfinding (chase/patrol) ─────────────────────────
        if threat.mode in ("chase", "patrol") and goal is not None:
            blocked = {(t.x, t.y) for t in world.threats if t is not threat}
            blocked |= {(m.x, m.y) for m in world.mothers if m.is_alive()}

            path = astar(
                (threat.x, threat.y), goal,
                world.grid_w, world.grid_h, blocked, moves_8=True
            )

            if path is None:
                next_x, next_y = best_step(
                    threat, goal, occupied_now,
                    world.grid_w, world.grid_h, world.RANDOM_MOVES
                )
            elif len(path) == 1:
                next_x, next_y = threat.x, threat.y
            else:
                next_x, next_y = path[1]
                if (next_x, next_y) in occupied_now and (next_x, next_y) != (threat.x, threat.y):
                    next_x, next_y = best_step(
                        threat, goal, occupied_now,
                        world.grid_w, world.grid_h, world.RANDOM_MOVES
                    )

        proposals[threat] = (next_x, next_y)

    intents = {
        "prev_pos": prev_positions,
        "target_child": target_child_map,
    }
    return proposals, intents


# ═══════════════════════════════════════════════════════════════════════
# Apply Threat Actions
# ═══════════════════════════════════════════════════════════════════════

def apply_threat_intents(world, intents):
    """Apply damage to children that threats have caught."""
    target_child_map = intents["target_child"]

    for threat in world.threats:
        child = target_child_map.get(threat)
        if child is None:
            continue

        if (child.is_alive()
                and not child.is_carried
                and (threat.x, threat.y) == (child.x, child.y)):
            child.injury += float(random.randint(5, 10))


# ═══════════════════════════════════════════════════════════════════════
# Internal Helpers
# ═══════════════════════════════════════════════════════════════════════

def _has_valid_child_target(children_seen):
    """Check if any visible child is alive and not carried."""
    if not children_seen:
        return False
    child, _ = min(children_seen, key=lambda pair: pair[1])
    return child is not None and child.is_alive() and not child.is_carried


def _mothers_to_positions(mothers_seen):
    """Extract (x, y) positions from perceived mother list."""
    return [(m.x, m.y) for m, _ in mothers_seen]


def _is_safe_from_mothers(threat, safe_distance):
    """Check if threat is safely far from all last-seen mother positions."""
    if not threat.last_seen_mothers:
        return True
    nearest = min(
        abs(threat.x - mx) + abs(threat.y - my)
        for mx, my in threat.last_seen_mothers
    )
    return nearest > safe_distance


def _flee_step(threat, mother_positions, world, occupied_now):
    """
    Greedy 1-step repulsion: choose neighbor maximizing total
    Manhattan distance from all known mother positions.
    """
    candidates = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            next_x = threat.x + dx
            next_y = threat.y + dy

            if not (0 <= next_x < world.grid_w and 0 <= next_y < world.grid_h):
                continue
            if (next_x, next_y) in occupied_now and (next_x, next_y) != (threat.x, threat.y):
                continue

            score = sum(
                abs(next_x - mx) + abs(next_y - my)
                for mx, my in mother_positions
            )
            candidates.append((score, next_x, next_y))

    if not candidates:
        return threat.x, threat.y

    candidates.sort(reverse=True)
    best_score, next_x, next_y = candidates[0]

    current_score = sum(
        abs(threat.x - mx) + abs(threat.y - my)
        for mx, my in mother_positions
    )

    if best_score <= current_score:
        return threat.x, threat.y

    return next_x, next_y


def _pick_patrol_goal(world, threat, max_tries=30):
    """Pick a random patrol destination."""
    for _ in range(max_tries):
        goal_x = random.randrange(world.grid_w)
        goal_y = random.randrange(world.grid_h)
        if (goal_x, goal_y) != (threat.x, threat.y):
            return (goal_x, goal_y)
    return (threat.x, threat.y)


def _pick_safe_patrol_goal(world, threat, mother_positions, max_tries=40):
    """Pick a patrol goal biased away from last-seen mother positions."""
    if not mother_positions:
        return _pick_patrol_goal(world, threat, max_tries=max_tries)

    best_goal = (threat.x, threat.y)
    best_score = -1

    for _ in range(max_tries):
        goal_x = random.randrange(world.grid_w)
        goal_y = random.randrange(world.grid_h)

        if (goal_x, goal_y) == (threat.x, threat.y):
            continue

        score = sum(
            abs(goal_x - mx) + abs(goal_y - my)
            for mx, my in mother_positions
        )

        if score > best_score:
            best_score = score
            best_goal = (goal_x, goal_y)

    return best_goal