from go2_patrol_patrol.patrol_sequence import (
    PHASE_DONE,
    PHASE_PATROL,
    PHASE_RETURN_HOME,
    REASON_FINISHED,
    REASON_LOW_BATTERY,
    PatrolSequence,
)
from go2_patrol_patrol.waypoints import Waypoint


def make_waypoints():
    return [
        Waypoint("home", 0.0, 0.0, 0.0, home=True),
        Waypoint("a", 2.0, 3.0, 0.0),
        Waypoint("b", -2.0, -3.0, 0.0),
    ]


def drive(sequence, limit=50):
    """按状态机自己的节奏走完全程，返回访问顺序。"""
    visited = []
    for _ in range(limit):
        waypoint = sequence.next_goal()
        if waypoint is None:
            break
        visited.append(waypoint.name)
        sequence.on_arrival(waypoint)
    return visited


def test_following_waypoint_is_the_next_point_in_order():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=False)
    assert sequence.next_goal().name == "home"
    assert sequence.following_waypoint().name == "a"
    sequence.on_arrival(sequence.next_goal())
    assert sequence.next_goal().name == "a"
    assert sequence.following_waypoint().name == "b"


def test_following_waypoint_wraps_to_first_point_on_endless_loops():
    sequence = PatrolSequence(make_waypoints(), loops=0, return_home=False)
    sequence.on_arrival(sequence.next_goal())
    sequence.on_arrival(sequence.next_goal())
    assert sequence.next_goal().name == "b"
    # 无限循环：b 之后回 home（点位表第一个）
    assert sequence.following_waypoint().name == "home"


def test_following_waypoint_is_none_on_last_loop_without_return_home():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=False)
    sequence.on_arrival(sequence.next_goal())
    sequence.on_arrival(sequence.next_goal())
    assert sequence.next_goal().name == "b"
    # 最后一圈的最后一个点，且不返航：没有下一段
    assert sequence.following_waypoint() is None


def test_following_waypoint_points_home_on_last_loop_with_return_home():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=True)
    sequence.on_arrival(sequence.next_goal())
    sequence.on_arrival(sequence.next_goal())
    assert sequence.next_goal().name == "b"
    assert sequence.following_waypoint().name == "home"


def test_following_waypoint_is_none_while_returning():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=True)
    sequence.on_arrival(sequence.next_goal())  # home
    sequence.on_arrival(sequence.next_goal())  # a
    sequence.on_arrival(sequence.next_goal())  # b -> 转入返航阶段
    assert sequence.returning
    assert sequence.next_goal().name == "home"
    assert sequence.following_waypoint() is None


def test_single_loop_visits_each_waypoint_then_stops():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=False)
    assert drive(sequence) == ["home", "a", "b"]
    assert sequence.phase == PHASE_DONE
    assert sequence.next_goal() is None


def test_return_home_adds_final_home_leg():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=True)
    assert drive(sequence) == ["home", "a", "b", "home"]
    assert sequence.phase == PHASE_DONE


def test_return_home_skipped_when_last_waypoint_is_home():
    """最后一点本来就是 home 时不要再发一次同样的目标。"""
    waypoints = [Waypoint("a", 2.0, 0.0, 0.0), Waypoint("home", 0.0, 0.0, 0.0, home=True)]
    sequence = PatrolSequence(waypoints, loops=1, return_home=True)
    assert drive(sequence) == ["a", "home"]
    assert sequence.phase == PHASE_DONE


def test_two_loops_then_return_home():
    sequence = PatrolSequence(make_waypoints(), loops=2, return_home=True)
    visited = drive(sequence, limit=100)
    assert visited == ["home", "a", "b", "home", "a", "b", "home"]
    assert sequence.loop_index == 2


def test_infinite_loops_never_finish():
    sequence = PatrolSequence(make_waypoints(), loops=0, return_home=True)
    for _ in range(30):
        waypoint = sequence.next_goal()
        assert waypoint is not None
        sequence.on_arrival(waypoint)
    assert sequence.phase == PHASE_PATROL
    assert sequence.loop_index == 10


def test_low_battery_returns_home_and_stops():
    sequence = PatrolSequence(make_waypoints(), loops=3, return_home=True)
    first = sequence.next_goal()
    sequence.on_arrival(first)
    assert sequence.request_return_home() is True
    assert sequence.phase == PHASE_RETURN_HOME
    assert sequence.return_reason == REASON_LOW_BATTERY
    assert drive(sequence) == ["home"]
    assert sequence.phase == PHASE_DONE
    # 低电量返航后不应再回到巡检节奏
    assert sequence.next_goal() is None
    assert sequence.loop_index == 0


def test_low_battery_during_return_home_keeps_single_leg():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=True)
    for _ in range(3):
        sequence.on_arrival(sequence.next_goal())
    assert sequence.phase == PHASE_RETURN_HOME
    assert sequence.return_reason == REASON_FINISHED
    assert sequence.request_return_home() is False
    assert sequence.return_reason == REASON_FINISHED
    assert drive(sequence) == ["home"]


def test_low_battery_after_done_is_ignored():
    sequence = PatrolSequence(make_waypoints(), loops=1, return_home=False)
    drive(sequence)
    assert sequence.request_return_home() is False
    assert sequence.phase == PHASE_DONE


def test_home_falls_back_to_first_waypoint():
    waypoints = [Waypoint("start", 1.0, 1.0, 0.0), Waypoint("x", 2.0, 2.0, 0.0)]
    sequence = PatrolSequence(waypoints, loops=1, return_home=True)
    assert sequence.home.name == "start"
    assert drive(sequence) == ["start", "x", "start"]


def test_empty_waypoints_is_immediately_done():
    sequence = PatrolSequence([], loops=1, return_home=True)
    assert sequence.done is True
    assert sequence.next_goal() is None


def test_copy_is_independent():
    sequence = PatrolSequence(make_waypoints(), loops=2, return_home=True)
    sequence.on_arrival(sequence.next_goal())
    clone = sequence.copy()
    clone.on_arrival(clone.next_goal())
    assert sequence.waypoint_index == 1
    assert clone.waypoint_index == 2
    assert sequence.loop_index == 0
    assert clone.loop_index == 0


def test_describe_mentions_loop_and_phase():
    sequence = PatrolSequence(make_waypoints(), loops=2, return_home=True)
    assert "圈 1/2" in sequence.describe()
    sequence.request_return_home()
    assert "return_home" in sequence.describe()
