
def replan_width(map, moves, robot, start, goal):
    robot_dimension = max(robot.width, robot.height)

    final_state = None
    opened = [start]
    closed = []

    while opened and final_state is None:  # or while we have enough time
        item = opened.pop()
        for move in moves:
            new_item = item.try_apply(map, move, robot)
            if new_item is not None:
                if new_item.dist_to(goal) < robot_dimension:
                    final_state = new_item
                    break
                else:
                    if not any(other_item.is_same_as(new_item) for other_item in opened):
                        opened.insert(0, new_item)
        closed.append(item)
    return final_state
