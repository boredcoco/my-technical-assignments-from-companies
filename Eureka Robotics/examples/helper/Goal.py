class Goal:
    def __init__(self, name: str, x: float, y: float, yaw: float):
        self.name = name
        self.x = x
        self.y = y
        self.yaw = yaw
        self.threshold = 0.05 # fixed for now
        self.threshold_yaw = 0.0872665 # in radians

    def check_within_goal(self, x, y, yaw):
        if abs(x - self.x) <= self.threshold and abs(y - self.y) <= self.threshold:
            if yaw is None or self.yaw is None or abs(self.yaw - yaw) <= self.threshold_yaw:
                return True
            return False
