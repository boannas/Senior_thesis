# core/policies/threat.py
import random
from core.sim.movement import in_bounds

def threat_policy_propose(world):
    """
    Propose threat moves based on start-of-step positions (no moves applied here).
    Returns:
      proposals: {threat_agent: (nx, ny)}
      intents: info needed to apply attacks after movement
    """
    perception_range = 2
    proposals = {}

    # Threat perceives only mother + child (your original design)
    threat_receive = world.mothers + world.children

    # store which child each threat is targeting (for attack after move)
    target_child = {}  # threat -> child

    # snapshot (optional; useful if you later update physiology for threats)
    prev_pos = {t: (t.x, t.y) for t in world.threats}

    for threat in world.threats:
        _, agents_perceived = threat.scan_perception(threat_receive, perception_range)

        # In your old code, agents_perceived[1] was children list
        # Assuming scan_perception returns (mothers_seen, children_seen, threats_seen) style,
        # but your old code suggests it returns [mothers, children]
        # We'll handle both safely:
        mothers_seen = agents_perceived[0] if len(agents_perceived) > 0 else []
        children_seen = agents_perceived[1] if len(agents_perceived) > 1 else []

        # 1) chase closest visible child (if not carried)
        if children_seen:
            # your old code assumes already sorted, but let's sort to be safe
            children_seen.sort(key=lambda t: t[1])
            child, _dist = children_seen[0]

            if child is not None and child.is_alive() and (not child.is_carried):
                dx, dy = threat.step_towards(child.x, child.y)
                nx, ny = threat.x + dx, threat.y + dy

                # keep in bounds
                if not in_bounds(nx, ny, world.grid_w, world.grid_h):
                    nx, ny = threat.x, threat.y

                proposals[threat] = (nx, ny)
                target_child[threat] = child
                continue

        # 2) fallback: random move
        dx, dy = random.choice(world.RANDOM_MOVES)
        nx, ny = threat.x + dx, threat.y + dy
        if not in_bounds(nx, ny, world.grid_w, world.grid_h):
            nx, ny = threat.x, threat.y

        proposals[threat] = (nx, ny)

    intents = {
        "prev_pos": prev_pos,
        "target_child": target_child,
    }
    return proposals, intents


def apply_threat_intents(world, intents):
    """
    Apply threat effects AFTER global movement resolution.
    Attacks happen if threat ends on the child's cell.
    """
    target_child = intents.get("target_child", {})

    for threat in world.threats:
        child = target_child.get(threat)
        if child is None:
            continue

        if child.is_alive() and (not child.is_carried) and (threat.x, threat.y) == (child.x, child.y):
            # your original: target_child.energy -= random.randint(5,10)
            child.energy -= random.randint(5, 10)