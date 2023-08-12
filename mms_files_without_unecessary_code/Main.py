# This is the modified version of the Drexels code for the micromouse https://github.com/james-ralph8555/DrexelMicromouse2020
# here all the unecessary API calls has been removed, which some added optimzation and easy config practice, so that the maze settings can be entered accordingly without any hassle in the main code

# here the the maze is always assumed to be square with the desitation at its center and start at any of the corners with three sides walls

import heapq
from collections import deque, namedtuple
from math import sqrt
import sys
import API

MAZE_WIDTH: int = API.mazeWidth()  # pyright: ignore[reportGeneralTypeIssues]
MAZE_HEIGHT: int = API.mazeHeight()  # pyright: ignore[reportGeneralTypeIssues]

DirectionsClass = namedtuple("Directions", "UP RIGHT DOWN LEFT")
Direction = DirectionsClass(0, 1, 2, 3)

return_chuncked_coords = lambda shunk_size=0, width=MAZE_WIDTH, height=MAZE_HEIGHT: int(
    (int(width / 2) + shunk_size + int(height / 2) + shunk_size) / 2
)


def manhattan_distance_explore(a, b, visited):
    """allows for path with visited nodes if neccessary, but strongly prefers unvisited nodes"""
    if visited[b[0]][b[1]]:
        return (abs(a[0] - b[0]) + abs(a[1] - b[1])) ** 2
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidian_distance_explore(a, b, visited):
    """allows for path with visited nodes if neccessary, but strongly prefers unvisited nodes"""
    if visited[b[0]][b[1]]:
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def aStar(maze_array, current_x, current_y, goal_x, goal_y, h_func, visited):
    """A* algorithm for pathfinding
    modified from code by Christian Careaga (MIT license) modifications: accessibility checking/removed numpy dependency/adapted for square grid/added support for any heuristic
    """
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    start = (current_x, current_y)
    goal = (goal_x, goal_y)
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: h_func(start, goal, visited)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.reverse()
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < len(maze_array) and 0 <= neighbor[1] < len(
                maze_array[1]
            ):
                if (
                    i == 0
                    and j == 1
                    and bool(
                        return_chuncked_coords(-4)
                        & maze_array[neighbor[0]][neighbor[1]]
                    )
                ):
                    continue
                if (
                    i == 0
                    and j == -1
                    and bool(1 & maze_array[neighbor[0]][neighbor[1]])
                ):
                    continue
                if (
                    i == 1
                    and j == 0
                    and bool(
                        return_chuncked_coords() & maze_array[neighbor[0]][neighbor[1]]
                    )
                ):
                    continue
                if (
                    i == -1
                    and j == 0
                    and bool(2 & maze_array[neighbor[0]][neighbor[1]])
                ):
                    continue
            else:
                continue

            tentative_g_score = gscore[current] + h_func(current, neighbor, visited)

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [
                i[1] for i in oheap
            ]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + h_func(neighbor, goal, visited)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False


def concat_bin(a, b):
    if a == MAZE_HEIGHT:
        return b
    if b == MAZE_WIDTH:
        return a
    return a | b


def mapping(
    maze_array, x, y, degmode, intersections
):  # see page 9 of the powerpoint to see how each cell is stored as a combinatation of 1,2,4,8 (0001, 0010, 0100, 1000)
    F = API.wallFront()
    R = API.wallRight()
    L = API.wallLeft()
    if F:
        if degmode is Direction.UP:
            if y < len(maze_array[0]) - 1:
                maze_array[x][y + 1] = concat_bin(maze_array[x][y + 1], 4)
            maze_array[x][y] = concat_bin(maze_array[x][y], 1)
        elif degmode is Direction.RIGHT:
            if x < len(maze_array) - 1:
                maze_array[x + 1][y] = concat_bin(maze_array[x + 1][y], 8)
            maze_array[x][y] = concat_bin(maze_array[x][y], 2)
        elif degmode is Direction.DOWN:
            if y > 0:
                maze_array[x][y - 1] = concat_bin(maze_array[x][y - 1], 1)
            maze_array[x][y] = concat_bin(maze_array[x][y], 4)
        elif degmode is Direction.LEFT:
            if x > 0:
                maze_array[x - 1][y] = concat_bin(maze_array[x - 1][y], 2)
            maze_array[x][y] = concat_bin(maze_array[x][y], 8)
    if R:
        if degmode is Direction.UP:
            if x < len(maze_array) - 1:
                maze_array[x + 1][y] = concat_bin(maze_array[x + 1][y], 8)
            maze_array[x][y] = concat_bin(maze_array[x][y], 2)
        elif degmode is Direction.RIGHT:
            if y > 0:
                maze_array[x][y - 1] = concat_bin(maze_array[x][y - 1], 1)
            maze_array[x][y] = concat_bin(maze_array[x][y], 4)
        elif degmode is Direction.DOWN:
            if x > 0:
                maze_array[x - 1][y] = concat_bin(maze_array[x - 1][y], 2)
            maze_array[x][y] = concat_bin(maze_array[x][y], 8)
        elif degmode is Direction.LEFT:
            if y < len(maze_array[0]) - 1:
                maze_array[x][y + 1] = concat_bin(maze_array[x][y + 1], 4)
            maze_array[x][y] = concat_bin(maze_array[x][y], 1)
    if L:
        if degmode is Direction.UP:
            if x > 0:
                maze_array[x - 1][y] = concat_bin(maze_array[x - 1][y], 2)
            maze_array[x][y] = concat_bin(maze_array[x][y], 8)
        elif degmode is Direction.RIGHT:
            if y < len(maze_array[0]) - 1:
                maze_array[x][y + 1] = concat_bin(maze_array[x][y + 1], 4)
            maze_array[x][y] = concat_bin(maze_array[x][y], 1)
        elif degmode is Direction.DOWN:
            if x < len(maze_array) - 1:
                maze_array[x + 1][y] = concat_bin(maze_array[x + 1][y], 8)
            maze_array[x][y] = concat_bin(maze_array[x][y], 2)
        elif degmode is Direction.LEFT:
            if y > 0:
                maze_array[x][y - 1] = concat_bin(maze_array[x][y - 1], 1)
            maze_array[x][y] = concat_bin(maze_array[x][y], 4)

    elif not F and not R and not L and maze_array[x][y] == 0:
        maze_array[x][y] = MAZE_HEIGHT or MAZE_WIDTH

    if (
        maze_array[x][y] == 1
        or maze_array[x][y] in (int(MAZE_HEIGHT / return_chuncked_coords()), int(MAZE_WIDTH / return_chuncked_coords()))  # type: ignore
        or maze_array[x][y] in (int(MAZE_HEIGHT / return_chuncked_coords(-4)), int(MAZE_WIDTH / return_chuncked_coords(-4)))  # type: ignore
        or maze_array[x][y] in (int(MAZE_HEIGHT / return_chuncked_coords(-6)), int(MAZE_WIDTH / return_chuncked_coords(-6)))  # type: ignore
        or maze_array[x][y] == MAZE_HEIGHT
        or MAZE_WIDTH
    ):
        intersections.append((x, y))

    return maze_array, intersections


def set_degmode(desired, cur, score):
    if desired == cur:
        return cur, score
    if desired == (cur + 1) % 4:  # modulo to loop back to 0 after 3
        API.turnRight()
        return desired, (score + 1)
    if desired == (cur - 1) % 4:
        API.turnLeft()
        return desired, (score + 1)
    API.turnRight()
    API.turnRight()
    return desired, (score + 2)


def BFS(maze_array, start, goal, visited):
    paths = []
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    q = deque()
    path = [start]
    q.append(path)
    while q:
        path = q[0]
        q.popleft()
        last = path[-1]
        if last == goal:
            paths.append(path)
        for i, j in neighbors:
            neighbor = (last[0] + i, last[1] + j)
            if (
                0 <= neighbor[0] < len(maze_array)
                and 0 <= neighbor[1] < len(maze_array[1])
                and last not in path[1:-1]
            ):
                if (
                    i == 0
                    and j == 1
                    and not bool(
                        return_chuncked_coords(-4)
                        & maze_array[neighbor[0]][neighbor[1]]
                    )
                ):
                    if visited[neighbor[0]][neighbor[1]]:
                        q.append(path + [neighbor])
                if (
                    i == 0
                    and j == -1
                    and not bool(1 & maze_array[neighbor[0]][neighbor[1]])
                ):
                    if visited[neighbor[0]][neighbor[1]]:
                        q.append(path + [neighbor])
                if (
                    i == 1
                    and j == 0
                    and not bool(
                        return_chuncked_coords() & maze_array[neighbor[0]][neighbor[1]]
                    )
                ):
                    if visited[neighbor[0]][neighbor[1]]:
                        q.append(path + [neighbor])
                if (
                    i == -1
                    and j == 0
                    and not bool(2 & maze_array[neighbor[0]][neighbor[1]])
                ):
                    if visited[neighbor[0]][neighbor[1]]:
                        q.append(path + [neighbor])
    best_path = []
    best_score = sys.maxsize
    for p in paths:
        cur_score = path_score(p)
        if cur_score < best_score:
            best_path = p
            best_score = cur_score
    return best_path, best_score


def move_to(current_x, current_y, degmode, path, maze_array, visited, score):
    dist = 0
    if path[0][0] == current_x and path[0][1] == current_y + 1:
        degmode, score = set_degmode(Direction.UP, degmode, score)
        for i, n in enumerate(path):
            if (
                n[0] == current_x
                and n[1] == current_y + 1 + i
                and visited[n[0]][n[1]]
                and not bool(4 & maze_array[n[0]][n[1]])
            ):
                dist += 1
            elif i == 0 and not bool(4 & maze_array[n[0]][n[1]]):
                dist = 1
                if not visited[n[0]][n[1]]:
                    break
            else:
                break
        if not API.wallFront():
            if dist <= 2:
                score += dist
            else:
                score += 2 + (dist - 2) / 2
            API.moveForward(dist)
            return current_x, current_y + dist, degmode, score
    if path[0][0] == current_x + 1 and path[0][1] == current_y:
        degmode, score = set_degmode(Direction.RIGHT, degmode, score)
        for i, n in enumerate(path):
            if (
                n[0] == current_x + 1 + i
                and n[1] == current_y
                and visited[n[0]][n[1]]
                and not bool(8 & maze_array[n[0]][n[1]])
            ):
                dist += 1
            elif i == 0 and not bool(8 & maze_array[n[0]][n[1]]):
                dist = 1
                if not visited[n[0]][n[1]]:
                    break
            else:
                break
        if not API.wallFront():
            if dist <= 2:
                score += dist
            else:
                score += 2 + (dist - 2) / 2
            API.moveForward(dist)
            return current_x + dist, current_y, degmode, score
    if path[0][0] == current_x and path[0][1] == current_y - 1:
        degmode, score = set_degmode(Direction.DOWN, degmode, score)
        for i, n in enumerate(path):
            if (
                n[0] == current_x
                and n[1] == current_y - 1 - i
                and visited[n[0]][n[1]]
                and not bool(1 & maze_array[n[0]][n[1]])
            ):
                dist += 1
            elif i == 0 and not bool(1 & maze_array[n[0]][n[1]]):
                dist = 1
                if not visited[n[0]][n[1]]:
                    break
            else:
                break
        if not API.wallFront():
            if dist <= 2:
                score += dist
            else:
                score += 2 + (dist - 2) / 2

            API.moveForward(dist)
            return current_x, current_y - dist, degmode, score
    if path[0][0] == current_x - 1 and path[0][1] == current_y:
        degmode, score = set_degmode(Direction.LEFT, degmode, score)
        for i, n in enumerate(path):
            if (
                n[0] == current_x - 1 - i
                and n[1] == current_y
                and visited[n[0]][n[1]]
                and not bool(2 & maze_array[n[0]][n[1]])
            ):
                dist += 1
            elif i == 0 and not bool(2 & maze_array[n[0]][n[1]]):
                dist = 1
                if not visited[n[0]][n[1]]:
                    break
            else:
                break
        if not API.wallFront():
            if dist <= 2:
                score += dist
            else:
                score += 2 + (dist - 2) / 2
            API.moveForward(dist)
            return current_x - dist, current_y, degmode, score


def path_score(path):
    cur_x = path[0][0]
    last_x = path[0][0]
    last_x2 = -1
    last_x3 = -1
    cur_y = path[0][1]
    last_y = path[0][1]
    last_y2 = -1
    last_y3 = -1

    score = -1
    for n in path[1 : len(path)]:
        cur_x = n[0]
        cur_y = n[1]
        if not (cur_x == last_x == last_x2) and not (cur_y == last_y == last_y2):
            score += 2
        elif (
            cur_x == last_x == last_x2 == last_x3
            or cur_y == last_y == last_y2 == last_y3
        ):
            score += 0.5
        else:
            score += 1
        last_x3 = last_x2
        last_y3 = last_y2
        last_x2 = last_x
        last_y2 = last_y
        last_x = cur_x
        last_y = cur_y
    return score


def update_viable(
    visited,
):
    """viable = explored + cells which have been visited on all 4 neighbors, but not visited themselves"""
    viable = [row[:] for row in visited]  # proper 2D array copying
    for i, _ in enumerate(viable):
        for j, _ in enumerate(viable):
            l = False
            r = False
            u = False
            d = False
            if i == 0:
                l = True
                if visited[i + 1][j]:
                    r = True
            elif i == 15:
                r = True
                if visited[i - 1][j]:
                    l = True
            else:
                if visited[i + 1][j] and visited[i - 1][j]:
                    l = True
                    r = True
            if j == 0:
                d = True
                if visited[i][j + 1]:
                    u = True
            elif j == 15:
                u = True
                if visited[i][j - 1]:
                    d = True
            else:
                if visited[i][j + 1] and visited[i][j - 1]:
                    u = True
                    d = True
            if l and r and u and d:
                viable[i][j] = True
    return viable


def main():
    maze_array = [[0 for j in range(MAZE_HEIGHT)] for i in range(MAZE_WIDTH)]  # type: ignore
    visited = [[False for j in range(MAZE_HEIGHT)] for i in range(MAZE_WIDTH)]  # type: ignore
    viable = [row[:] for row in visited]
    intersections = []
    FINISH_X = 8
    FINISH_Y = 8
    START_X = 0
    START_Y = 0
    current_x = START_X
    current_y = START_Y
    degmode = Direction.UP
    State_Class = namedtuple("State", "start_to_goal goal_to_start final_run")
    State = State_Class(0, 1, 2)
    state = State.start_to_goal
    visited[START_X][START_Y] = True
    got_best_score = False
    total_score = 0
    encountered_wall = False
    while True:
        if (
            return_chuncked_coords(-1) <= current_x <= return_chuncked_coords()
            and return_chuncked_coords(-1) <= current_y <= return_chuncked_coords()
        ):
            if state == State.start_to_goal:
                state = State.goal_to_start
                FINISH_X = current_x
                FINISH_Y = current_y
            if state is State.final_run:
                exit()
        elif (
            any(
                visited[return_chuncked_coords(-1) : return_chuncked_coords(1)][
                    return_chuncked_coords(-1) : return_chuncked_coords(1)
                ]
            )
            and state is State.start_to_goal
        ):
            state = State.goal_to_start
        if (
            state is State.goal_to_start
            and current_x == START_X
            and current_y == START_Y
        ):
            state = State.final_run
        maze_array, intersections = mapping(
            maze_array, current_x, current_y, degmode, intersections
        )
        if state is State.start_to_goal or state is State.final_run:
            goto_x = FINISH_X
            goto_y = FINISH_Y
        elif state is State.goal_to_start:
            goto_x = START_X
            goto_y = START_Y
        while (
            current_x != goto_x or current_y != goto_y
        ):  # pyright: ignore[reportUnboundVariable]s
            if state is State.start_to_goal or state is State.goal_to_start:
                maze_array, intersections = mapping(
                    maze_array, current_x, current_y, degmode, intersections
                )
                if not encountered_wall:
                    if not all(
                        [
                            all(
                                [
                                    ((cell == 0) != (cell == MAZE_WIDTH))
                                    for cell in row[1:-1]
                                ]
                            )
                            for row in maze_array[1:-1]
                        ]
                    ):  # checks if there are walls in array; != is xor for two booleans
                        encountered_wall = True
                if encountered_wall:
                    path = aStar(
                        maze_array,
                        current_x,
                        current_y,
                        goto_x,  # pyright: ignore[reportUnboundVariable]
                        goto_y,  # pyright: ignore[reportUnboundVariable]
                        euclidian_distance_explore,
                        visited,
                    )
                else:
                    path = aStar(
                        maze_array,
                        current_x,
                        current_y,
                        goto_x,  # pyright: ignore[reportUnboundVariable]
                        goto_y,  # pyright: ignore[reportUnboundVariable]
                        manhattan_distance_explore,
                        visited,
                    )
            elif state is State.final_run:
                path, _ = BFS(
                    maze_array, (current_x, current_y), (FINISH_X, FINISH_Y), viable
                )
                if not got_best_score:
                    num_visited = 0
                    for c in [row.count(True) for row in visited]:
                        num_visited += c
                    got_best_score = True
            if path:  # pyright: ignore[reportUnboundVariable]
                if (current_x, current_y) in path:
                    path.remove((current_x, current_y))
                current_x, current_y, degmode, total_score = move_to(
                    current_x, current_y, degmode, path, maze_array, viable, total_score
                )
                visited[current_x][current_y] = True
                viable = update_viable(visited)
                if (
                    return_chuncked_coords(-1) <= current_x <= return_chuncked_coords()
                    and return_chuncked_coords(-1)
                    <= current_y
                    <= return_chuncked_coords()
                ):
                    break


if __name__ == "__main__":
    main()
