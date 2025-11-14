"""
Simple entry point to run the grid world with agent.

Threats are configured in configs/base.yaml and will move randomly each step.
You can modify threat positions in the YAML file or override them here.
"""
from ui.pygame_viewer import main

if __name__ == "__main__":
    # Option 1: Use threats from config file (configs/base.yaml)
    main()
    
    # Option 2: Override threat positions directly in code
    # threat_positions = [
    #     [5, 5],   # Threat at position (5, 5) - will move randomly
    #     [9, 9],   # Threat at position (9, 9) - will move randomly
    #     [2, 2],   # Additional threat at position (2, 2)
    #     [12, 12], # Additional threat at position (12, 12)
    # ]
    # main(threat_positions=threat_positions)
