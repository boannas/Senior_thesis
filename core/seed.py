"""
Centralized random seed initialization.
Call init_seed() once at program startup to ensure reproducibility.
All other modules should import `rng` from here instead of creating their own.
"""

import random
import numpy as np

DEFAULT_SEED = 42

# Global numpy Generator for modules that need it (e.g., agent weight init)
rng = np.random.default_rng(DEFAULT_SEED)


def init_seed(seed=DEFAULT_SEED):
    """
    Initialize all random number generators with the given seed.
    Must be called once at program startup before any simulation runs.
    """
    random.seed(seed)
    np.random.seed(seed)
    global rng
    rng = np.random.default_rng(seed)
