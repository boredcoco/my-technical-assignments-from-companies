from helper.PixelMap import PixelMap
from helper.BoundingBox import BoundingBox
from helper.Path import Path

class PathFinder:
    def __init__(self, pixelMap):
        self.pixelMap = pixelMap

    def find_best_move(self, x, y, goal):
        path = Path()
        # try to move direct to goal
        #self, xmin: float, ymin: float, xmax: float, ymax: float
        bounding_box = BoundingBox(min((x, goal.x)), min((y, goal.y)), max((x, goal.x)), max((y, goal.y)))
        has_obstacle = self.pixelMap.has_obstacle_in_bounding_box(bounding_box)
        if not has_obstacle:
            path.add_step(goal.x, goal.y)
            return path

        # use bfs to find a path
        path_lst = self.pixelMap.get_path_to_dest(x, y, goal)
        for (path_x, path_y) in path_lst:
            path.add_step(path_x, path_y)
        return path
