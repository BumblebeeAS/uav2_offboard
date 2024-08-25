import numpy as np

class MotionGenerator:
    def __init__(self, max_vel, accel) -> None:
        self.max_vel = max_vel
        self.accel = accel
        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.target_pose = np.array([0.0, 0.0, 0.0])
        self.current_vel = np.array([0.0, 0.0, 0.0])
        self.braking = False

    def set_target(self, target: list[float]):
        self.braking = False
        self.target_pose = np.array(target)
    
    def calc_dist_to_target(self):
        dist = np.linalg.norm(self.target_pose - self.current_pose)
        return dist

    def calc_dir_unit_vec(self):
        dir_vec = (self.target_pose - self.current_pose)
        norm = np.linalg.norm(dir_vec)
        if norm == 0:
            return np.array([0, 0, 0])
        return dir_vec / norm

    def calc_braking_dist(self):
        current_vel_magnitude = np.linalg.norm(self.current_vel)
        return current_vel_magnitude * current_vel_magnitude / (2 * self.accel)

    def calc_next_vel(self, delta_time):
        braking_dist = self.calc_braking_dist()
        dist_to_target = self.calc_dist_to_target()

        if self.braking or dist_to_target <= braking_dist:
            self.braking = True
            next_vel_mag = max(np.linalg.norm(self.current_vel) - self.accel * delta_time, 0.0)
        else:
            next_vel_mag = min(np.linalg.norm(self.current_vel) + self.accel * delta_time, self.max_vel)
        
        return next_vel_mag * self.calc_dir_unit_vec()

    def update_current_vel(self, current_vel: list[float]):
        self.current_vel = np.array(current_vel)

    def update_current_pose(self, current_pose: list[float]):
        self.current_pose = np.array(current_pose)

    def update_position(self, delta_time):
        # print(f"current_vel: {self.current_vel}")
        self.current_pose += self.current_vel * delta_time