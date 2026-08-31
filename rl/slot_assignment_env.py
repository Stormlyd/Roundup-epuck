"""Pure-Python contract for slot-assignment reinforcement-learning environments."""

from dataclasses import dataclass, replace
import math
from types import MappingProxyType
from typing import Tuple


FIELD_WIDTH = 1.460
FIELD_HEIGHT = 0.914
MARGIN = 0.057
HEADING_COUNT = 24
PERMUTATION_COUNT = 6
ACTION_COUNT = 144
PERMUTATIONS = ((0, 1, 2), (0, 2, 1), (1, 0, 2), (1, 2, 0), (2, 0, 1), (2, 1, 0))
PHASE_RADII = MappingProxyType({"PURSUIT": 0.31, "SURROUND": 0.24, "CAPTURE": 0.1224})


@dataclass(frozen=True)
class RobotState:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class SlotState:
    phase: str
    target: RobotState
    target_vx: float
    target_vy: float
    pursuers: Tuple[RobotState, RobotState, RobotState]
    previous_action: int = -1


@dataclass(frozen=True)
class RewardTerms:
    captured: bool = False
    capture_progress_delta: float = 0
    elapsed_seconds: float = 0
    path_length_metres: float = 0
    crossings: float = 0
    cbf_interventions: float = 0
    invalid_transition: bool = False


@dataclass(frozen=True)
class RewardWeights:
    captured: float = 100
    capture_progress_delta: float = 10
    elapsed_seconds: float = -1
    path_length_metres: float = -2
    crossings: float = -5
    cbf_interventions: float = -2
    switched: float = -1
    invalid_transition: float = -100


def _finite_number(value):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


def _valid_robot(robot):
    return (
        isinstance(robot, RobotState)
        and all(_finite_number(value) for value in (robot.x, robot.y, robot.yaw))
        and 0 <= robot.x <= FIELD_WIDTH
        and 0 <= robot.y <= FIELD_HEIGHT
    )


def validate_state(slot_state):
    """Return whether a state is finite and lies within the physical field."""
    return (
        isinstance(slot_state, SlotState)
        and type(slot_state.phase) is str
        and slot_state.phase in PHASE_RADII
        and _valid_robot(slot_state.target)
        and _finite_number(slot_state.target_vx)
        and _finite_number(slot_state.target_vy)
        and isinstance(slot_state.pursuers, tuple)
        and len(slot_state.pursuers) == 3
        and all(_valid_robot(robot) for robot in slot_state.pursuers)
        and isinstance(slot_state.previous_action, int)
        and not isinstance(slot_state.previous_action, bool)
        and -1 <= slot_state.previous_action < ACTION_COUNT
    )


def decode_action(action):
    if not isinstance(action, int) or isinstance(action, bool):
        raise TypeError("action must be an integer")
    if not 0 <= action < ACTION_COUNT:
        raise ValueError("action must be in [0, 143]")
    return action // PERMUTATION_COUNT, PERMUTATIONS[action % PERMUTATION_COUNT]


def encode_action(heading_index, permutation_tuple):
    if not isinstance(heading_index, int) or isinstance(heading_index, bool):
        raise TypeError("heading_index must be an integer")
    if not 0 <= heading_index < HEADING_COUNT:
        raise ValueError("heading_index must be in [0, 23]")
    if not isinstance(permutation_tuple, tuple):
        raise TypeError("permutation_tuple must be a tuple")
    if any(type(index) is not int for index in permutation_tuple) or permutation_tuple not in PERMUTATIONS:
        raise ValueError("permutation_tuple must be authoritative")
    return heading_index * PERMUTATION_COUNT + PERMUTATIONS.index(permutation_tuple)


def encode_observation(slot_state):
    if not validate_state(slot_state):
        raise ValueError("invalid slot state")
    phase = tuple(float(slot_state.phase == name) for name in ("PURSUIT", "SURROUND", "CAPTURE"))
    target = (
        slot_state.target.x / FIELD_WIDTH,
        slot_state.target.y / FIELD_HEIGHT,
        max(-0.03, min(0.03, slot_state.target_vx)) / 0.03,
        max(-0.03, min(0.03, slot_state.target_vy)) / 0.03,
    )
    pursuers = tuple(
        item
        for robot in slot_state.pursuers
        for item in (robot.x / FIELD_WIDTH, robot.y / FIELD_HEIGHT, math.sin(robot.yaw), math.cos(robot.yaw))
    )
    previous = 0.0 if slot_state.previous_action == -1 else (slot_state.previous_action + 1) / ACTION_COUNT
    return phase + target + pursuers + (previous,)


def goals_for_action(slot_state, action):
    if not validate_state(slot_state):
        raise ValueError("invalid slot state")
    heading, permutation = decode_action(action)
    theta = heading * (2 * math.pi / HEADING_COUNT)
    radius = PHASE_RADII[slot_state.phase]
    ring = tuple(
        (
            slot_state.target.x + radius * math.cos(theta + 2 * math.pi * index / 3),
            slot_state.target.y + radius * math.sin(theta + 2 * math.pi * index / 3),
        )
        for index in range(3)
    )
    goals = tuple(ring[index] for index in permutation)
    # Keep strict comparisons aligned with C++ ringInsideBounds training/execution parity.
    if any(not (MARGIN <= x <= FIELD_WIDTH - MARGIN and MARGIN <= y <= FIELD_HEIGHT - MARGIN) for x, y in goals):
        return None
    return goals


def _valid_terms(terms):
    return (
        isinstance(terms, RewardTerms)
        and isinstance(terms.captured, bool)
        and isinstance(terms.invalid_transition, bool)
        and all(
            _finite_number(value)
            for value in (
                terms.capture_progress_delta,
                terms.elapsed_seconds,
                terms.path_length_metres,
                terms.crossings,
                terms.cbf_interventions,
            )
        )
        and all(
            value >= 0
            for value in (
                terms.elapsed_seconds,
                terms.path_length_metres,
                terms.crossings,
                terms.cbf_interventions,
            )
        )
    )


def _valid_weights(weights):
    return isinstance(weights, RewardWeights) and all(
        _finite_number(value)
        for value in (
            weights.captured,
            weights.capture_progress_delta,
            weights.elapsed_seconds,
            weights.path_length_metres,
            weights.crossings,
            weights.cbf_interventions,
            weights.switched,
            weights.invalid_transition,
        )
    )


def score_reward(terms, weights=None, switched=False):
    if not isinstance(switched, bool):
        raise TypeError("switched must be a boolean")
    if weights is None:
        weights = RewardWeights()
    if not _valid_weights(weights):
        raise ValueError("invalid reward weights")
    if not _valid_terms(terms):
        raise ValueError("invalid reward terms")
    if terms.invalid_transition:
        return float(weights.invalid_transition)
    try:
        reward = float(
            weights.captured * terms.captured
            + weights.capture_progress_delta * terms.capture_progress_delta
            + weights.elapsed_seconds * terms.elapsed_seconds
            + weights.path_length_metres * terms.path_length_metres
            + weights.crossings * terms.crossings
            + weights.cbf_interventions * terms.cbf_interventions
            + weights.switched * switched
        )
    except OverflowError as error:
        raise ValueError("reward must be finite") from error
    if not math.isfinite(reward):
        raise ValueError("reward must be finite")
    return reward


class SlotAssignmentEnv:
    """Stateful transition wrapper; ``transition`` receives ``(state, goals)``."""

    def __init__(self, transition, weights=None):
        self._transition = transition
        self._weights = RewardWeights() if weights is None else weights
        if not callable(transition):
            raise TypeError("transition must be callable")
        if not _valid_weights(self._weights):
            raise ValueError("invalid reward weights")
        self._state = None

    def reset(self, slot_state):
        if not validate_state(slot_state):
            raise ValueError("invalid slot state")
        self._state = slot_state
        return encode_observation(slot_state)

    def _invalid_result(self, observation):
        return observation, float(self._weights.invalid_transition), True, {"invalid": True}

    def step(self, action):
        if self._state is None:
            raise RuntimeError("reset must be called before step")
        previous_observation = encode_observation(self._state)
        try:
            goals = goals_for_action(self._state, action)
        except (TypeError, ValueError):
            return self._invalid_result(previous_observation)
        if goals is None:
            return self._invalid_result(previous_observation)
        response = self._transition(self._state, goals)
        try:
            next_state, terms, terminated = response
        except (TypeError, ValueError):
            return self._invalid_result(previous_observation)
        if not validate_state(next_state) or not _valid_terms(terms) or not isinstance(terminated, bool):
            return self._invalid_result(previous_observation)
        if terms.invalid_transition:
            return self._invalid_result(previous_observation)
        switched = self._state.previous_action != -1 and action != self._state.previous_action
        try:
            reward = score_reward(terms, self._weights, switched)
        except ValueError:
            return self._invalid_result(previous_observation)
        self._state = replace(next_state, previous_action=action)
        return encode_observation(self._state), reward, terminated, {"invalid": False}
