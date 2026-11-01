import random
from collections import deque
from PIL import Image


class PixelMap:
    def __init__(self, origin, resolution, img_path="map.png", threshold=230):
        img = Image.open(img_path).convert("L")
        self.W, self.H = img.size
        self.pixels = img.load()
        self.obstacle = [[False] * self.W for _ in range(self.H)]
        self.origin = origin
        self.resolution = resolution

        for r in range(self.H):
            for c in range(self.W):
                if self.pixels[c, r] > threshold:
                    self.obstacle[r][c] = True

        self._safe = None   # lazily built safety map

    def has_obstacle_in_bounding_box(self, bounding_box):
        """Assume bounding box has real-world coordinates."""
        r_min, c_min = self.world_to_pixel(bounding_box.xmin, bounding_box.ymin)
        r_max, c_max = self.world_to_pixel(bounding_box.xmax, bounding_box.ymax)

        # top left corner is 0, 0 increases downward rather than upward
        for r in range(r_max, r_min + 1):
            for c in range(c_min, c_max + 1):
                if self.obstacle[r][c]:
                    return True
        return False

    def add_obstacle_world_coordinates(self, world_x, world_y, radius=0):
        """
        Mark the cell at world (world_x, world_y) as an obstacle.
        If radius > 0, mark all cells within `radius` world units (circular).
        Invalidates the cached safety map so subsequent BFS calls see the change.
        Returns True if any cell was newly marked, False otherwise.
        """
        r0, c0 = self.world_to_pixel(world_x, world_y)
        if not self._in_bounds(r0, c0):
            return False

        changed = False

        if radius <= 0:
            if not self.obstacle[r0][c0]:
                self.obstacle[r0][c0] = True
                changed = True
        else:
            rr = int(radius / self.resolution)
            r_lo, r_hi = max(0, r0 - rr), min(self.H - 1, r0 + rr)
            c_lo, c_hi = max(0, c0 - rr), min(self.W - 1, c0 + rr)
            for r in range(r_lo, r_hi + 1):
                for c in range(c_lo, c_hi + 1):
                    wx, wy = self.pixel_to_world(r, c)
                    if (wx - world_x) ** 2 + (wy - world_y) ** 2 <= radius ** 2:
                        if not self.obstacle[r][c]:
                            self.obstacle[r][c] = True
                            changed = True

        if changed:
            self._safe = None
        return True

    # ---------- coordinate helpers (unchanged) ----------
    def pixel_to_world(self, r, c):
        x = self.origin[0] + (c + 0.5) * self.resolution
        y = self.origin[1] + (self.H - 1 - r + 0.5) * self.resolution
        return x, y

    def world_to_pixel(self, x, y):
        c = (x - self.origin[0]) / self.resolution - 0.5
        r = (self.H - 0.5) - (y - self.origin[1]) / self.resolution
        return int(r), int(c)

    # ---------- internal helpers ----------
    def _in_bounds(self, r, c):
        return 0 <= r < self.H and 0 <= c < self.W

    def _ensure_safety_map(self):
        """self._safe[r][c] is True if cell (r, c) is NOT adjacent to an obstacle."""
        if self._safe is not None:
            return
        safe = [[True] * self.W for _ in range(self.H)]
        for r in range(self.H):
            for c in range(self.W):
                if not self.obstacle[r][c]:
                    continue
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = r + dr, c + dc
                        if self._in_bounds(nr, nc):
                            safe[nr][nc] = False
        self._safe = safe

    def _reconstruct(self, parent, end):
        path = []
        cur = end
        while cur is not None:
            path.append(self.pixel_to_world(*cur))
            cur = parent[cur]
        path.reverse()
        return path

    def _nearest_free_cell(self, r, c, max_radius=40):
        if self._in_bounds(r, c) and not self.obstacle[r][c]:
            return r, c
        for rad in range(1, max_radius + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    if max(abs(dr), abs(dc)) != rad:
                        continue
                    nr, nc = r + dr, c + dc
                    if self._in_bounds(nr, nc) and not self.obstacle[nr][nc]:
                        return nr, nc
        return None

    # ---------- BFS ----------
    def get_path_to_dest(self, x, y, goal):
        path_lst = self._bfs(x, y, goal)
        if path_lst is None:
            return None
        return self._compress_path(path_lst)

    def _compress_path(self, path):
        if len(path) <= 2:
            return path
        compressed = [path[0]]
        curr_direction = 0 # -1 for x, 1 for y

        for i in range(1, len(path) - 1):
            prev_p = compressed[-1]
            curr_p = path[i]

            # if you are horizontall or vertically aligned with prev_p
            # get rid of prev_p and replace with curr_p
            if prev_p[0] == curr_p[0] and curr_direction == -1:
                compressed.pop()
                compressed.append(curr_p)
                curr_direction = -1
            elif prev_p[-1] == curr_p[-1] and curr_direction == 1:
                compressed.pop()
                compressed.append(curr_p)
                curr_direction = 1
            else:
                compressed.append(curr_p)
                if prev_p[-1] == curr_p[-1]:
                    curr_direction = 1
                elif prev_p[0] == curr_p[0]:
                    curr_direction = -1
                else:
                    curr_direction = 0
        # Always add the final destination
        compressed.append(path[-1])
        return compressed

    def _bfs(self, x, y, goal):
        start = self._nearest_free_cell(*self.world_to_pixel(x, y))
        if start is None:
            return None

        end = self._nearest_free_cell(*self.world_to_pixel(goal.x, goal.y))
        if end is None:
            return None

        parent = {start: None}
        q = deque([start])

        while q:
            cur = q.popleft()
            if cur == end:
                return self._reconstruct(parent, cur)

            r, c = cur
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (r + dr, c + dc)
                if nb in parent:
                    continue
                nr, nc = nb
                if not self._in_bounds(nr, nc):
                    continue
                if self.obstacle[nr][nc]:
                    continue
                parent[nb] = cur
                q.append(nb)

        return [(random.randint(-1, 3), random.randint(-1, 3))]
