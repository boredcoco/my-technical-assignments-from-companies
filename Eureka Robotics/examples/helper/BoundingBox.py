import math

class BoundingBox:
    def __init__(self, xmin: float, ymin: float, xmax: float, ymax: float):
        self.xmin = float(xmin)
        self.ymin = float(ymin)
        self.xmax = float(xmax)
        self.ymax = float(ymax)

    def __repr__(self) -> str:
        return (f"BoundingBox(xmin={self.xmin:.3f}, ymin={self.ymin:.3f}, "
                f"xmax={self.xmax:.3f}, ymax={self.ymax:.3f})")

    def __str__(self) -> str:
        return (f"BoundingBox(xmin={self.xmin:.3f}, ymin={self.ymin:.3f}, "
                f"xmax={self.xmax:.3f}, ymax={self.ymax:.3f})")

    def overlaps(self, other) -> bool:
        if (self.xmax < other.xmin or other.xmax < self.xmin):
            return False
        if self.ymax < other.ymin or other.ymax < self.ymin:
            return False
        return True

    def min_euclidean_distance(self, other):
        dx = max(0.0, other.xmin - self.xmax, self.xmin - other.xmax)
        dy = max(0.0, other.ymin - self.ymax, self.ymin - other.ymax)
        return math.hypot(dx, dy)
