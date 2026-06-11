"""
Movement resolution for the grid world.
Handles pathfinding fallback (best greedy step) and multi-agent
collision resolution (destination conflicts + swap conflicts).
"""


def in_bounds(x, y, grid_width, grid_height):
    """Check if (x, y) is within the grid boundaries."""
    return 0 <= x < grid_width and 0 <= y < grid_height


def best_step(agent, goal_xy, occupied_cells, grid_width, grid_height, possible_moves):
    """
    Greedy one-step move toward a goal, avoiding occupied cells.

    Parameters
    ----------
    agent : Agent
        The agent to move.
    goal_xy : tuple(int, int)
        Target (x, y) position.
    occupied_cells : set of tuple
        Cells currently occupied by other agents.
    grid_width, grid_height : int
        Grid dimensions.
    possible_moves : list of tuple(int, int)
        Relative (dx, dy) moves the agent can make.

    Returns
    -------
    tuple(int, int)
        Best next (x, y) position.
    """
    goal_x, goal_y = goal_xy

    candidates = [(agent.x, agent.y)]
    for dx, dy in possible_moves:
        next_x, next_y = agent.x + dx, agent.y + dy
        if in_bounds(next_x, next_y, grid_width, grid_height):
            candidates.append((next_x, next_y))

    def movement_score(cell):
        next_x, next_y = cell

        # Heavily penalize occupied cells
        if (next_x, next_y) in occupied_cells and (next_x, next_y) != (agent.x, agent.y):
            return 10**9

        # Manhattan distance to goal
        distance = abs(next_x - goal_x) + abs(next_y - goal_y)

        # Discourage staying in place
        if (next_x, next_y) == (agent.x, agent.y):
            distance += 10**9

        return distance

    return min(candidates, key=movement_score)


def resolve_and_apply_moves(agents, proposals, grid_width, grid_height):
    """
    Resolve movement conflicts and apply moves for all agents.

    Handles:
    1. Destination conflicts — multiple agents claim the same cell
    2. Swap conflicts — two agents try to swap positions
    3. Final application — move non-blocked agents

    Parameters
    ----------
    agents : list of Agent
        All agents attempting to move.
    proposals : dict
        {agent: (target_x, target_y)} proposed moves.
    grid_width, grid_height : int
        Grid dimensions.
    """
    current_positions = {agent: (agent.x, agent.y) for agent in agents}

    def agent_priority(agent):
        """Priority for conflict resolution (lower = higher priority)."""
        agent_id = getattr(agent, "id", None)
        if agent_id is None:
            agent_id = getattr(agent, "mother_id", None)
        if agent_id is None:
            agent_id = getattr(agent, "child_id", None)

        if agent_id is None:
            return (3, id(agent))  # Fallback: use object id
        return (0, str(agent_id))  # Explicit id gets higher priority

    # --- Step 1: Detect destination conflicts ---
    destination_map = {}
    for agent, destination in proposals.items():
        destination_map.setdefault(destination, []).append(agent)

    blocked_agents = set()

    for destination, claimers in destination_map.items():
        if len(claimers) > 1:
            winner = min(claimers, key=agent_priority)
            for agent in claimers:
                if agent is not winner:
                    blocked_agents.add(agent)

    # --- Step 2: Detect swap conflicts ---
    for agent_a, dest_a in proposals.items():
        if agent_a in blocked_agents:
            continue
        for agent_b, dest_b in proposals.items():
            if agent_b is agent_a or agent_b in blocked_agents:
                continue
            if dest_a == current_positions[agent_b] and dest_b == current_positions[agent_a]:
                winner = min([agent_a, agent_b], key=agent_priority)
                loser = agent_b if winner is agent_a else agent_a
                blocked_agents.add(loser)

    # --- Step 3: Apply moves ---
    occupied = set(current_positions.values())
    for agent, (next_x, next_y) in proposals.items():
        if agent in blocked_agents:
            continue

        next_x = max(0, min(grid_width - 1, next_x))
        next_y = max(0, min(grid_height - 1, next_y))

        if (next_x, next_y) in occupied and (next_x, next_y) != current_positions[agent]:
            continue

        occupied.discard(current_positions[agent])
        agent.x, agent.y = next_x, next_y
        occupied.add((next_x, next_y))