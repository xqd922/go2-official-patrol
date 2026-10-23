from go2_patrol_driver.safety import (
    ACTION_EMERGENCY,
    ACTION_GO,
    ACTION_NO_SCAN,
    ACTION_SLOW,
    ACTION_STOP,
    front_action,
    is_fresh,
    scan_is_fresh,
)

LIMITS = dict(emergency_distance=0.20, stop_distance=0.35, slow_distance=0.60)


def test_scan_is_fresh_needs_a_recent_scan():
    assert scan_is_fresh(9_500_000_000, 10_000_000_000, 1.0) is True
    assert scan_is_fresh(8_000_000_000, 10_000_000_000, 1.0) is False


def test_never_received_a_scan_counts_as_stale():
    """点云断流 / DDS 掉线时最容易出事：没有扫描 ≠ 前方畅通。"""
    assert scan_is_fresh(None, 10_000_000_000, 1.0) is False


def test_front_action_maps_distance_to_action():
    assert front_action(0.10, **LIMITS) == ACTION_EMERGENCY
    assert front_action(0.20, **LIMITS) == ACTION_EMERGENCY
    assert front_action(0.30, **LIMITS) == ACTION_STOP
    assert front_action(0.35, **LIMITS) == ACTION_STOP
    assert front_action(0.50, **LIMITS) == ACTION_SLOW
    assert front_action(0.60, **LIMITS) == ACTION_GO
    assert front_action(float("inf"), **LIMITS) == ACTION_GO


def test_stale_scan_stops_instead_of_silently_disabling_the_guard():
    assert front_action(float("inf"), fresh=False, **LIMITS) == ACTION_NO_SCAN
    assert front_action(0.10, fresh=False, **LIMITS) == ACTION_NO_SCAN


def test_require_scan_can_be_turned_off():
    """留在离线夹具 / 无雷达台架上跑时用，行为与改动前一致。"""
    assert front_action(float("inf"), fresh=False, require_scan=False, **LIMITS) == ACTION_GO


def test_is_fresh_is_the_same_rule_for_any_periodic_input():
    """/cmd_vel 断流看门狗与 /scan 看门狗共用同一条"新鲜"判定。"""
    assert is_fresh(9_900_000_000, 10_000_000_000, 0.5) is True
    assert is_fresh(9_000_000_000, 10_000_000_000, 0.5) is False
    assert is_fresh(None, 10_000_000_000, 0.5) is False
    assert scan_is_fresh(9_900_000_000, 10_000_000_000, 0.5) is is_fresh(
        9_900_000_000, 10_000_000_000, 0.5
    )
