import math
import pytest

from my_robot_perception.goal_checker_node import distance_to_goal, GoalLatch


def test_distance_to_goal_zero():
    assert distance_to_goal(1.0, 2.0, 1.0, 2.0) == pytest.approx(0.0)


def test_distance_to_goal_unit():
    assert distance_to_goal(0.0, 0.0, 3.0, 4.0) == pytest.approx(5.0)


def test_distance_to_goal_negative_coords():
    assert distance_to_goal(-1.0, -1.0, 2.0, 3.0) == pytest.approx(5.0)


def test_goal_latch_starts_false():
    latch = GoalLatch(threshold=0.3)
    assert latch.reached is False


def test_goal_latch_transitions_to_true_within_threshold():
    latch = GoalLatch(threshold=0.3)
    just_transitioned = latch.update(robot_x=1.0, robot_y=1.0, goal_x=1.1, goal_y=1.0)
    assert just_transitioned is True
    assert latch.reached is True


def test_goal_latch_stays_false_outside_threshold():
    latch = GoalLatch(threshold=0.3)
    just_transitioned = latch.update(robot_x=0.0, robot_y=0.0, goal_x=1.0, goal_y=1.0)
    assert just_transitioned is False
    assert latch.reached is False


def test_goal_latch_one_shot_only():
    latch = GoalLatch(threshold=0.3)
    first = latch.update(0.0, 0.0, 0.0, 0.0)
    second = latch.update(0.0, 0.0, 0.0, 0.0)
    third = latch.update(10.0, 10.0, 0.0, 0.0)
    assert first is True
    assert second is False
    assert third is False
    assert latch.reached is True


def test_goal_latch_at_exact_threshold_is_not_reached():
    latch = GoalLatch(threshold=0.3)
    just_transitioned = latch.update(robot_x=0.0, robot_y=0.0, goal_x=0.3, goal_y=0.0)
    assert just_transitioned is False
    assert latch.reached is False
