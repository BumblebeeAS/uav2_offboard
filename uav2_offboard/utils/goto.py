from dataclasses import dataclass

from bb_uav_msgs.action import GoToPosition, Takeoff


@dataclass
class GeneralGoal:
    x: float
    y: float
    z: float
    x_threshold: float
    y_threshold: float
    z_threshold: float
    relative: bool = True

    @staticmethod
    def from_position_goal(position_goal: GoToPosition.Goal) -> "GeneralGoal":
        return GeneralGoal(
            x=position_goal.x,
            y=position_goal.y,
            z=position_goal.z,
            x_threshold=position_goal.x_threshold,
            y_threshold=position_goal.y_threshold,
            z_threshold=position_goal.z_threshold,
            relative=position_goal.relative,
        )

    @staticmethod
    def from_takeoff_goal(takeoff_goal: Takeoff.Goal) -> "GeneralGoal":
        return GeneralGoal(
            x=0.0,
            y=0.0,
            z=-takeoff_goal.altitude,
            x_threshold=takeoff_goal.x_threshold,
            y_threshold=takeoff_goal.y_threshold,
            z_threshold=takeoff_goal.z_threshold,
            relative=True,
        )
