"""
Entity classes for non-agent objects in the simulation world.
Currently includes: Food.
"""


class Entity:
    """Base class for all entities in the grid world."""

    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

    def __repr__(self):
        return f"{self.name}({self.x}, {self.y})"


class Food(Entity):
    """Food entity that can be collected by agents to restore energy."""

    def __init__(self, x, y):
        super().__init__("Food", x, y)
        self.collected = False

    def collect(self):
        """Mark this food as collected (will be removed from world)."""
        self.collected = True
