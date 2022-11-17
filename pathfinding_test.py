from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# pathfinding matrix. 1 = walkable cells and 0 = obstacles
matrix = [
  [1, 1, 1],
  [1, 0, 1],
  [1, 1, 0]
]

# create grid from the matrix
grid = Grid(matrix=matrix)

# create start and end point
start = grid.node(0, 0)
end = grid.node(2, 2)

# create instance of finder. Disable diagonal movement
# finds path from start -> end, and amount of runs required to find a path
finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
path, runs = finder.find_path(start, end, grid)

# print result
print('operations:', runs, 'path length:', len(path))
print(grid.grid_str(path=path, start=start, end=end))
print((path))
