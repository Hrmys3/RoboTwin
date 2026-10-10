"""Scene contract and an oracle adapter for the first VPC experiment.

The oracle deliberately uses simulator state. A future vision adapter should
produce the same ObjectEstimate fields from camera observations.
"""

from dataclasses import dataclass

import numpy as np
import sapien

from envs.utils.actor_utils import Actor


@dataclass(frozen=True)
class ObjectEstimate:
    name: str
    pose: tuple[float, ...]  # world xyz + wxyz quaternion
    confidence: float
    model_id: int


@dataclass(frozen=True)
class SceneEstimate:
    breads: tuple[ObjectEstimate, ...]
    basket: ObjectEstimate
    robot: "RobotEstimate"


@dataclass(frozen=True)
class RobotEstimate:
    left_ee_pose: tuple[float, ...]
    right_ee_pose: tuple[float, ...]
    left_gripper_open: bool
    right_gripper_open: bool


def pose_vector(actor: Actor) -> tuple[float, ...]:
    pose = actor.get_pose()
    return tuple(float(v) for v in np.r_[pose.p, pose.q])


class _BiasedEntity:
    """Expose a position-biased pose while keeping the real physics entity."""

    def __init__(self, entity, position_bias):
        self._entity = entity
        self._position_bias = np.asarray(position_bias, dtype=float)

    def get_pose(self):
        real = self._entity.get_pose()
        return sapien.Pose(real.p + self._position_bias, real.q)

    def __getattr__(self, name):
        return getattr(self._entity, name)


class EstimatedActor(Actor):
    """Adapter for RoboTwin's grasp/place primitives; never changes physics."""

    def __init__(self, actor: Actor, position_bias):
        self.actor = _BiasedEntity(actor.actor, position_bias)
        self.config = actor.config

    @property
    def position_error_m(self) -> tuple[float, ...]:
        return tuple(float(v) for v in self.actor._position_bias)


class OracleSceneProvider:
    """Read privileged poses; optionally apply fixed per-object position errors."""

    def __init__(self, task, position_sigma_m: float, seed: int):
        if position_sigma_m < 0:
            raise ValueError("position_sigma_m must be nonnegative")
        rng = np.random.default_rng(seed)
        self.breads = tuple(
            EstimatedActor(actor, rng.normal(0, position_sigma_m, 3))
            for actor in task.bread
        )
        self.basket = EstimatedActor(
            task.breadbasket, rng.normal(0, position_sigma_m, 3)
        )
        self.task = task

    def estimate(self) -> SceneEstimate:
        return SceneEstimate(
            breads=tuple(
                ObjectEstimate("bread", pose_vector(actor), 1.0, int(model_id))
                for actor, model_id in zip(self.breads, self.task.bread_id)
            ),
            basket=ObjectEstimate(
                "basket", pose_vector(self.basket), 1.0, int(self.task.basket_id)
            ),
            robot=RobotEstimate(
                left_ee_pose=tuple(float(v) for v in self.task.robot.get_left_ee_pose()),
                right_ee_pose=tuple(float(v) for v in self.task.robot.get_right_ee_pose()),
                left_gripper_open=bool(self.task.robot.is_left_gripper_open()),
                right_gripper_open=bool(self.task.robot.is_right_gripper_open()),
            ),
        )
