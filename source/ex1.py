import itertools
import search
import math
import copy

ids = ["0", "0"]


class QItem:
    def __init__(self, row, col, dist):
        self.row = row
        self.col = col
        self.dist = dist

    def __repr__(self):
        return f"QItem({self.row}, {self.col}, {self.dist})"


def min_distance(grid):
    source = QItem(0, 0, 0)

    # Finding the source to start from
    for row in range(len(grid)):
        for col in range(len(grid[row])):
            if grid[row][col] == 's':
                source.row = row
                source.col = col
                break

    # To maintain location visit status
    visited = [[False for _ in range(len(grid[0]))]
               for _ in range(len(grid))]

    # applying BFS on matrix cells starting from source
    queue = []
    queue.append(source)
    visited[source.row][source.col] = True
    while len(queue) != 0:
        source = queue.pop(0)

        # Destination found;
        if (grid[source.row][source.col] == 'd'):
            return source.dist

        # moving up
        if is_valid(source.row - 1, source.col, grid, visited):
            queue.append(QItem(source.row - 1, source.col, source.dist + 1))
            visited[source.row - 1][source.col] = True

        # moving down
        if is_valid(source.row + 1, source.col, grid, visited):
            queue.append(QItem(source.row + 1, source.col, source.dist + 1))
            visited[source.row + 1][source.col] = True

        # moving left
        if is_valid(source.row, source.col - 1, grid, visited):
            queue.append(QItem(source.row, source.col - 1, source.dist + 1))
            visited[source.row][source.col - 1] = True

        # moving right
        if is_valid(source.row, source.col + 1, grid, visited):
            queue.append(QItem(source.row, source.col + 1, source.dist + 1))
            visited[source.row][source.col + 1] = True

    return float("inf")


# checking where move is valid or not
def is_valid(x, y, grid, visited):
    if ((x >= 0 and y >= 0) and
            (x < len(grid) and y < len(grid[0])) and
            (grid[x][y] != 'I') and (visited[x][y] == False)):
        return True
    return False


def flatten(t):
    return set([item for sublist in t for item in sublist])


def valid_action(action):
    picked_packages = [a[2] for a in action if a[0] == "pick up"]
    if len(picked_packages) != len(set(picked_packages)):
        return False
    return True


def manhattan_distance(loc1, loc2):
    return abs(loc1[0] - loc2[0]) + abs(loc1[1] - loc2[1])


def euclidean_distance(loc1, loc2):
    return ((loc1[0] - loc2[0]) ** 2 + (loc1[1] - loc2[1]) ** 2) ** 0.5


def loc_of_client_per_package(package_name, state):
    for client in state[2]:
        if package_name in client[2]:
            return client[1]
    return 0, 0


def loc_of_drone_per_package(package_name, state):
    for drone in state[0]:
        if package_name in drone[2]:
            return drone[1]
    return 0, 0


def min_drone_dist(package_loc, state, distance_function):
    dists = [distance_function(package_loc, drone[1]) for drone in state[0] if len(drone[2]) < 2]
    if len(dists) > 0:
        return min(dists)
    return 2


def action_score(action):
    return sum([2 if a[0] == "deliver" else 1 if a[0] == "pick up" else 0 for a in action])


def dict_to_initial_state(dict):
    # filter redundant packages
    required_packages = flatten([list(val["packages"]) for val in dict["clients"].values()])

    drones_loc = tuple([(name, loc, ()) for name, loc in dict["drones"].items()])
    packages_loc = tuple(
        [(name, loc, None) for name, loc in dict["packages"].items() if name in required_packages])
    clients_loc_and_rem_packages = tuple(
        [(name, 0, client_data["packages"]) for name, client_data in dict["clients"].items()])

    return drones_loc, packages_loc, clients_loc_and_rem_packages


class DroneProblem(search.Problem):
    """This class implements a medical problem according to problem description file"""

    def __init__(self, initial):
        """Don't forget to implement the goal test
        You should change the initial to your own representation.
        search.Problem.__init__(self, initial) creates the root node"""
        initial_state = dict_to_initial_state(initial)
        search.Problem.__init__(self, initial_state)
        self.map = initial["map"]
        self.grid_rows = len(self.map)
        self.grid_cols = len(self.map[0])
        self.clients = initial["clients"]
        self.packages_data = {}
        self.packages_loc = initial["packages"]
        self.add_client_avg_loc_and_packages_data()
        self.num_drones = len(initial["drones"])
        self.distances = self.shortest_path_dist()
        self.drones_initial_data = initial["drones"]

    def dist_func(self, loc1, loc2):
        """
        return the grid shortest path distance from loc1 to loc2 with referring to walls
        :param loc1: source
        :param loc2: destination
        :return:
        """
        return self.distances[(loc1, loc2)]

    def dist_drone_to_package(self, drone_name, package, state):
        if drone_name == "fictive":
            return self.grid_cols + self.grid_rows
        else:
            loc1 = None
            for drone_data in state[0]:
                if drone_data[0] == drone_name:
                    loc1 = drone_data[1]
            loc2 = self.packages_loc[package]
            return self.dist_func(loc1, loc2)

    def match_score(self, permutation, state):
        return sum([self.dist_drone_to_package(pair[0], pair[1], state) for pair in permutation])

    def shortest_path_dist(self):
        distances_dict = {}
        all_locs = list(itertools.product(range(self.grid_rows), range(self.grid_cols)))
        for source in all_locs:
            for destination in all_locs:
                if source == destination:
                    distances_dict[(source, destination)] = 0
                else:
                    grid = copy.deepcopy(self.map)
                    grid[source[0]][source[1]] = 's'
                    grid[destination[0]][destination[1]] = 'd'
                    distances_dict[(source, destination)] = min_distance(grid)

        return distances_dict

    def add_client_avg_loc_and_packages_data(self):
        for client, client_data in self.clients.items():
            for package in client_data["packages"]:
                self.packages_data[package] = client
            row_loc, col_loc = 0, 0
            for loc in client_data["path"]:
                row_loc += loc[0]
                col_loc += loc[1]
            len_path = len(client_data["path"])
            client_data["avg_loc"] = (row_loc / len_path, col_loc / len_path)

    def min_client_path_dist(self, package_name, package_loc, distance_function):
        return min([distance_function(package_loc, client_loc) for client_loc in
                    self.clients[self.packages_data[package_name]]["path"]])

    def is_in_grid(self, loc):
        """
        :param loc:
        :return:
        """
        return 0 <= loc[0] < self.grid_rows and 0 <= loc[1] < self.grid_cols and self.map[loc[0]][loc[1]] == 'P'

    def client_next_loc(self, client_name, current_loc):
        return (current_loc + 1) % len(self.clients[client_name]["path"])

    def single_drone_actions(self, clients_data, drone_data, all_packages_data):
        """

        :param clients_data:
        :param drone_data:
        :param all_packages_data:
        :return:
        """
        drone_num_packages_carried = len(drone_data[2])
        actions = [("wait", drone_data[0])]

        # move grid actions
        for step in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            new_loc = (drone_data[1][0] + step[0], drone_data[1][1] + step[1])
            if self.is_in_grid(new_loc):
                actions.append(("move", drone_data[0], new_loc))

        # pick up actions
        if drone_num_packages_carried < 2:
            for package in all_packages_data:
                if drone_data[1] == package[1]:
                    # if package is carried by other drone than package loc is null -> False
                    actions.append(("pick up", drone_data[0], package[0]))

        # deliver actions
        if drone_num_packages_carried > 0:
            for package in drone_data[2]:
                for client in clients_data:
                    if drone_data[1] == self.clients[client[0]]["path"][client[1]] and package in client[2]:
                        actions.append(("deliver", drone_data[0], client[0], package))

        return actions

    # Exe required functions
    def actions(self, state):
        """Returns all the actions that can be executed in the given
        state. The result should be a tuple (or other iterable) of actions
        as defined in the problem description file"""
        if self.num_drones <= 2:
            drones_actions = []
            for drone_data in state[0]:
                drones_actions.append(self.single_drone_actions(state[2], drone_data, state[1]))

            actions = [sub_action for sub_action in itertools.product(*drones_actions) if valid_action(sub_action)]
            return actions
        else:
            drones_actions = []
            for drone_data in state[0][:2]:
                drones_actions.append(self.single_drone_actions(state[2], drone_data, state[1]))
            left_over_drones = [("wait", drone[0]) for drone in state[0][2:]]
            actions = [tuple(list(sub_action) + left_over_drones) for sub_action in itertools.product(*drones_actions)
                       if valid_action(sub_action)]
            # actions = [sub_action for sub_action in itertools.product(*drones_actions) if valid_action(sub_action)]
            return actions

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""

        drones_loc_dict = {}
        for drone_data in state[0]:
            drones_loc_dict[drone_data[0]] = {"loc": drone_data[1], "packages": list(drone_data[2])}

        packages_loc_dict = {}
        for package_data in state[1]:
            packages_loc_dict[package_data[0]] = {"loc": package_data[1], "drone_carries": package_data[2]}

        clients_loc_and_rem_packages_dict = {}
        for client_data in state[2]:
            clients_loc_and_rem_packages_dict[client_data[0]] = {"loc": client_data[1],
                                                                 "rem_packages": list(client_data[2])}

        # update data by sub-actions
        for sub_action in action:
            if sub_action[0] == "move":
                drones_loc_dict[sub_action[1]]["loc"] = sub_action[2]
            elif sub_action[0] == "pick up":
                drones_loc_dict[sub_action[1]]["packages"].append(sub_action[2])
                packages_loc_dict[sub_action[2]]["loc"] = None
                packages_loc_dict[sub_action[2]]["drone_carries"] = sub_action[1]
            elif sub_action[0] == "deliver":
                drones_loc_dict[sub_action[1]]["packages"].remove(sub_action[3])
                del packages_loc_dict[sub_action[3]]
                clients_loc_and_rem_packages_dict[sub_action[2]]["rem_packages"].remove(sub_action[3])

        # update data by clients paths
        for client, client_data in clients_loc_and_rem_packages_dict.items():
            client_data["loc"] = self.client_next_loc(client, client_data["loc"])

        # transforms dicts to state's tuples
        drones_loc_tuple = tuple([(drone, drone_data["loc"], tuple(drone_data["packages"])) for drone, drone_data in
                                  drones_loc_dict.items()])

        packages_loc_tuple = tuple(
            [(package, package_data["loc"], package_data["drone_carries"]) for package, package_data in
             packages_loc_dict.items()])

        clients_loc_and_rem_packages_tuple = tuple(
            [(client, client_data["loc"], tuple(client_data["rem_packages"])) for client, client_data in
             clients_loc_and_rem_packages_dict.items()])

        # print("#" * 120)
        # print("initial_state: ", state)
        # print("action: ", action)
        # print("result state:  ", (drones_loc_tuple, packages_loc_tuple, clients_loc_and_rem_packages_tuple))
        return drones_loc_tuple, packages_loc_tuple, clients_loc_and_rem_packages_tuple

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        if len(state[1]) == 0:
            return True
        return False

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
            state can be accessed via node.state)
            and returns a goal distance estimate"""
        state = node.state
        if len(state[1]) == 0:
            return 0
        punishment = 0

        if self.num_drones <= 2 and node.action is not None and sum([1 for i, drone in enumerate(state[0]) if
                                                                     len(drone[2]) == 0 and node.action[i][
                                                                         0] == 'wait']) > 0 and sum(
            [1 for package in state[1] if package[1] is not None]) > 0:
            return float('inf')

        if node.parent is not None:
            if node.parent.parent is not None:
                prev_state = node.parent.state
                second_prev_state = node.parent.parent.state
                for i in range(len(state[0])):
                    if state[0][i][1] == second_prev_state[0][i][1] and state[0][i][1] != prev_state[0][i][1]:
                        punishment += 10
                        break
            current = node
            while current.parent is not None:
                if state == current.parent.state:
                    punishment += 10
                    break
                current = current.parent

        sum_of_dists = sum(
            [min_drone_dist(p[1], state, self.dist_func) + self.min_client_path_dist(p[0], p[1], self.dist_func) + 2 if
             p[1] is not None else self.min_client_path_dist(p[0], loc_of_drone_per_package(p[0], state),
                                                             self.dist_func) for p in state[1]])
        result = sum_of_dists / self.num_drones

        return result + math.log2(1 + node.depth) * len(state[1]) ** 1.26 + punishment


def create_drone_problem(game):
    return DroneProblem(game)
