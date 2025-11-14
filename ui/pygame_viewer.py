"""
Grid World with Agent - Pygame Viewer
Main entry point for running the grid world simulation with Mother, Child, Food, Threat, and Nest entities.
"""
import pygame
import sys
from pathlib import Path

# Add parent directory to path to import modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.world import World
from func.function_code import (
    load_config, draw_grid, draw_mother, draw_child, 
    draw_food, draw_threat, draw_nest
)


def main(threat_positions=None):
    """
    Main function to run the gridworld simulation.
    
    Args:
        threat_positions: Optional list of threat positions [(x, y), ...]. 
                         If None, threats will be loaded from config file.
    """
    # Load configuration
    config_path = Path(__file__).parent.parent / "configs" / "base.yaml"
    cfg = load_config(str(config_path))
    
    # Extract configuration
    grid_w = cfg["grid"]["width"]
    grid_h = cfg["grid"]["height"]
    cell_px = cfg["grid"]["cell_px"]
    fps = cfg["fps"]
    seed = cfg["seed"]
    
    # Agent positions
    mother_start = cfg["mother"]["start"]
    child_start = cfg["child"]["start"]
    
    
    # Entity positions
    food_positions = cfg["food"].get("positions", [])
    # Use provided threat_positions or load from config
    if threat_positions is None:
        threat_positions = cfg["threats"].get("positions", [])
    nest_position = cfg["nest"].get("position", None)
    
    # Colors
    bg_color = tuple(cfg["colors"]["bg"])
    grid_color = tuple(cfg["colors"]["grid"])
    mother_color = tuple(cfg["colors"]["mother"])
    child_color = tuple(cfg["colors"]["child"])
    food_color = tuple(cfg["colors"]["food"])
    threat_color = tuple(cfg["colors"]["threat"])
    nest_color = tuple(cfg["colors"]["nest"])
    outline_color = tuple(cfg["colors"]["outline"])
    
    # Initialize pygame
    pygame.init()
    
    # Calculate window size
    window_w = grid_w * cell_px
    window_h = grid_h * cell_px
    screen = pygame.display.set_mode((window_w, window_h))
    pygame.display.set_caption("Grid World: Mother, Child, Food, Threat, and Nest")
    clock = pygame.time.Clock()
    
    # Create world with all entities
    world = World(
        grid_w, grid_h,
        mother_start=mother_start,
        child_start=child_start,
        food_positions=food_positions,
        threat_positions=threat_positions,
        nest_position=nest_position,
        seed=seed
    )
    
    # Action mapping: 0=stay, 1=up, 2=down, 3=left, 4=right
    mother_action_keys = {
        pygame.K_UP: 1,
        pygame.K_DOWN: 2,
        pygame.K_LEFT: 3,
        pygame.K_RIGHT: 4,
        pygame.K_w: 1,  # W for up
        pygame.K_s: 2,  # S for down
        pygame.K_a: 3,  # A for left
        pygame.K_d: 4,  # D for right
    }
    
    # Child action keys (IJKL or numpad)
    child_action_keys = {
        pygame.K_i: 1,  # I for up
        pygame.K_k: 2,  # K for down
        pygame.K_j: 3,  # J for left
        pygame.K_l: 4,  # L for right
    }
    
    print("Grid World with Multiple Entities")
    print("=" * 50)
    print(f"Grid size: {grid_w}x{grid_h}")
    print(f"Mother starting position: ({world.mother.x}, {world.mother.y})")
    print(f"Child starting position: ({world.child.x}, {world.child.y})")
    print(f"Number of food items: {len(world.foods)}")
    print(f"Number of threats: {len(world.threats)}")
    print(f"Nest position: {world.nest.get_position() if world.nest else 'None'}")
    print("\nControls:")
    print("  Mother: Arrow keys or WASD")
    print("  Child: IJKL keys (when not carried)")
    print("  ESC or Close window: Exit")
    print("\nGameplay:")
    print("  - Mother can pick up child when on same cell")
    print("  - Mother can collect food when on same cell")
    print("  - Mother can drop child/food at nest")
    print("  - Avoid threats!")
    print("=" * 50)
    
    # Main game loop
    running = True
    
    while running:
        mother_action = 0
        child_action = None
        
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in mother_action_keys:
                    mother_action = mother_action_keys[event.key]
                elif event.key in child_action_keys and not world.child.is_carried:
                    child_action = child_action_keys[event.key]
        
        # Step world only if there's an action (discrete movement)
        if mother_action != 0 or child_action is not None:
            world.step(mother_action, child_action)
        
        # Render in order (background to foreground)
        draw_grid(screen, grid_w, grid_h, cell_px, bg_color, grid_color, outline_color)
        
        # Draw nest first (background)
        if world.nest:
            draw_nest(screen, world.nest, cell_px, nest_color, outline_color)
        
        # Draw food
        for food in world.foods:
            draw_food(screen, food, cell_px, food_color, outline_color)
        
        # Draw threats
        for threat in world.threats:
            draw_threat(screen, threat, cell_px, threat_color, outline_color)
        
        # Draw child (before mother so mother appears on top)
        draw_child(screen, world.child, cell_px, child_color, outline_color)
        
        # Draw mother (on top)
        draw_mother(screen, world.mother, cell_px, mother_color, outline_color)
        
        # Update display
        pygame.display.flip()
        clock.tick(fps)
    
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()

