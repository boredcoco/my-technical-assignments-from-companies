class Path:
    def __init__(self):
        self.steps = [] # comprises of tuple (x, y)

    def add_step(self, x, y):
        self.steps.append((x, y))

    def get_steps(self):
        return self.steps
