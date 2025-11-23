"""
Entity classes for Food, Threat, and Nest
"""

class Entity:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

    def __repr__(self):
        return f"{self.name}({self.x}, {self.y})"


class Food(Entity):
    """Food entity that can be collected by agents"""
    def __init__(self, x, y):
        super().__init__("Food", x, y)
        # self.x = x
        # self.y = y
        self.collected = False
    
    def get_position(self):
        return (self.x, self.y)
    
    def collect(self):
        """Mark food as collected"""
        self.collected = True

class Nest:
    """Nest entity - safe location for agents"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def get_position(self):
        return (self.x, self.y)
    
    def is_at_position(self, x, y):
        """Check if nest is at given position"""
        return self.x == x and self.y == y


