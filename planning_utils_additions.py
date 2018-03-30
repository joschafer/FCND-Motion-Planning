import numpy as np

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

# From previous A* work
def prune_path(path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    i=0
    while i < len(pruned_path) - 2:
        start = point(pruned_path[i])
        middle = point(pruned_path[i+1])
        end = point(pruned_path[i+2])
        if collinearity_check(start, middle, end):
            # remove middle
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1

    return pruned_path
