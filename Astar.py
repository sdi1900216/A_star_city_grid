import heapq

terrain_cost = {
    'R': 1,
    'H': 0.5,
    'B': 9999,
    'P': 2,
    'W': 9999,
    'S': 0,
    'G': 0
}

movements = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def manhattan_distance(start, goal):
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])


def a_star_search(grid, start, goal):
    rows, cols = len(grid), len(grid[0])

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal) / 2}

    expanded_nodes = 0
    fringe_order = []
    expanded_set = set()

    while open_set:
        current_f, current = heapq.heappop(open_set)
        fringe_order.append(current)

        heuristic = manhattan_distance(current, goal) / 2
        current_cost = g_score[current]
        print(f"Current Node: {current}")
        print(f"Heuristic (h): {heuristic}")
        print(f"Cost so far (g): {current_cost}")
        print(f"Expanded Nodes: {expanded_set}")
        print(f"Fringe: {[node for _, node in open_set]}")

        if current == goal:
            return g_score[goal], expanded_nodes, fringe_order

        expanded_nodes += 1
        expanded_set.add(current)

        for dx, dy in movements:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                cost = terrain_cost[grid[neighbor[0]][neighbor[1]]]

                if cost == float('inf'):
                    continue

                tentative_g_score = g_score[current] + cost

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal) / 2

                    if neighbor not in (item[1] for item in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return float('inf'), expanded_nodes, fringe_order


city_grid = [
    ['S', 'R', 'R', 'R', 'B', 'W', 'R', 'H', 'H', 'H'],
    ['R', 'B', 'B', 'R', 'H', 'H', 'R', 'R', 'B', 'H'],
    ['R', 'P', 'P', 'R', 'B', 'R', 'R', 'R', 'B', 'R'],
    ['R', 'R', 'R', 'R', 'W', 'R', 'P', 'P', 'R', 'R'],
    ['R', 'R', 'B', 'R', 'R', 'R', 'H', 'H', 'R', 'B'],
    ['B', 'W', 'R', 'P', 'P', 'R', 'B', 'R', 'R', 'R'],
    ['P', 'P', 'R', 'R', 'R', 'R', 'R', 'R', 'B', 'B'],
    ['R', 'B', 'R', 'R', 'R', 'W', 'H', 'H', 'R', 'R'],
    ['R', 'R', 'R', 'R', 'B', 'R', 'R', 'R', 'B', 'R'],
    ['H', 'H', 'H', 'B', 'B', 'R', 'R', 'G', 'R', 'R'],
]

start_pos = (0, 0)
goal_pos = (9, 7)

total_cost, nodes_expanded, fringe_order = a_star_search(city_grid, start_pos, goal_pos)

print(f"\nΣυνολικό κόστος διαδρομής: {total_cost}")
print(f"Αριθμός επεκτεινόμενων κόμβων: {nodes_expanded}")
print("Σειρά κόμβων από τη λίστα 'σύνορο':")
for node in fringe_order:
    print(node)
