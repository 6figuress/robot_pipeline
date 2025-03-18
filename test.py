import copy
 
# default joint weights (Look into this?)

JOINT_WEIGHT_ARR = [1,1,1,1,1,1]
 
# A position is an array with 6 (rad) angles, corresponding to each joint
 
# A solution is composed of a suite of postition
 
# A x, y, z coordinate can be attained by multiple position
 
# Class to store a full trajectory

class TrajectoryClass:

    trajectory: list[list[float]] = []

    weight: float = 0.0
 
    def __init__(self, traj: list[list[float]], joint_weights = JOINT_WEIGHT_ARR):

        self.trajectory = traj

        self.joint_weights = joint_weights
 
    def __lt__(self, other):

         #This is useful to help sorting the trajectories by weight

         return self.weight < other.weight

    def __str__(self):

       return f"Trajectory: {str(self.trajectory)}, Weight: {str(self.weight)}"
 
    def eval_weight(self):

        self.weight = 0

        # Computes the weight of the trajectory

        if len(self.trajectory) > 1:

            for i in range(len(self.trajectory)-1):

                for j in range(len(self.trajectory[i])):

                    self.weight += abs(self.trajectory[i][j] - self.trajectory[i+1][j]) * self.joint_weights[j]
 
    def add_point(self, pos: list[float]):

        return TrajectoryClass(self.trajectory + (pos))
 
 
# Function which lets us delete trajectories that are suboptimal

# If two path lead to the same joint state, the one with less cost is ALWAYS better (at that point)

def kill_traj(traj: list[TrajectoryClass]):

    best_trajs = {}

    for i in traj:

        i.eval_weight()

        last_position = tuple(i.trajectory[-1])  # Ensure it's hashable

        if last_position not in best_trajs or best_trajs[last_position].weight > i.weight:

            best_trajs[last_position] = i  # Keep the trajectory with the lowest weight

    return list(best_trajs.values())
 
 
def best_first_search(nodes):

    trajectories = []

    # Compute all possibilites for node 0 to node 1

    for i in nodes[0]:

        for j in nodes[1]:

            trajectories.append(TrajectoryClass([i, j]))

    # Dont explore suboptimal paths

    trajectories = kill_traj(trajectories)

    trajectories.sort() # Sorting so best solution is always to the start

    while len(trajectories[0].trajectory) < len(nodes):

        for i in nodes[len(trajectories[0].trajectory)]:

            # Compute possibilities for next nodes, only on best path so far

            newTraj = copy.deepcopy(trajectories[0].trajectory)

            newTraj.append(i)

            trajectories.append(TrajectoryClass(newTraj))

        # Replace the best path with the new path

        trajectories.pop(0)

        trajectories = kill_traj(trajectories)

        trajectories.sort() # Sorting so best solution is always to the start

    return trajectories[0]
 
# run by using best_first_search()
 
joints: list[list[list[float]]] = [[[]]]
 
best_first_search(joints)
 