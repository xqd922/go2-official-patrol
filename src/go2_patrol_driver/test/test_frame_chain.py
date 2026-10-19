from go2_patrol_driver.frame_chain import (
    MODE_ALWAYS,
    MODE_AUTO,
    MODE_NEVER,
    is_ours,
    is_target_link,
    normalize_frame,
    normalize_mode,
    publish_due,
    should_publish,
)


def test_normalize_mode_accepts_known_values_and_ignores_case():
    assert normalize_mode("always") == MODE_ALWAYS
    assert normalize_mode(" NEVER ") == MODE_NEVER
    assert normalize_mode("auto") == MODE_AUTO


def test_normalize_mode_falls_back_to_auto():
    """认不出来的一律按最保守的 auto：宁可探测后补链，也不要无条件抢着发。"""
    assert normalize_mode("") == MODE_AUTO
    assert normalize_mode(None) == MODE_AUTO
    assert normalize_mode("alwayss") == MODE_AUTO


def test_never_never_publishes():
    assert should_publish(MODE_NEVER, False, 100.0, 5.0) is False
    assert should_publish(MODE_NEVER, True, 100.0, 5.0) is False


def test_always_publishes_right_away():
    assert should_publish(MODE_ALWAYS, False, 0.0, 5.0) is True
    # always 是现场明确要求"就由我来补"，即使看到别处也在发也照发
    assert should_publish(MODE_ALWAYS, True, 0.0, 5.0) is True


def test_auto_waits_out_the_probe_window():
    assert should_publish(MODE_AUTO, False, 0.0, 5.0) is False
    assert should_publish(MODE_AUTO, False, 4.9, 5.0) is False
    assert should_publish(MODE_AUTO, False, 5.0, 5.0) is True


def test_auto_stands_down_forever_once_someone_else_publishes():
    assert should_publish(MODE_AUTO, True, 5.0, 5.0) is False
    assert should_publish(MODE_AUTO, True, 999.0, 5.0) is False


def test_is_ours_matches_only_stamps_we_actually_sent():
    """按 stamp 精确匹配：不能按时间窗。

    本节点自己按 publish_period 重发，上一帧离 now 很近；若用"相差 <0.3s 就算自己"，
    同机外部发布者的时间戳同样≈now，会被一起吞掉 → external_seen 永远为假 →
    auto 模式不回退，继续和机器狗抢发同一段 TF（真机表现为 Nav2 报多个父节点）。
    """
    mine = [1_000_000_000, 2_000_000_000]
    assert is_ours(1_000_000_000, mine) is True
    assert is_ours(2_000_000_000, mine) is True
    assert is_ours(1_200_000_000, mine) is False   # 离自己那帧只有 0.2s，但不是自己发的
    assert is_ours(1_000_000_000, []) is False
    assert is_ours(None, mine) is False


def test_is_target_link_requires_both_frames():
    """只比子帧名会把 odom→base_link 当成"这一环已有人负责"，而链其实还是断的。"""
    assert is_target_link("base_footprint", "base_link", "base_footprint", "base_link") is True
    assert is_target_link("odom", "base_link", "base_footprint", "base_link") is False
    assert is_target_link("", "base_link", "base_footprint", "base_link") is False
    assert is_target_link("base_footprint", "base_footprint", "base_footprint", "base_link") is False


def test_is_target_link_ignores_leading_slash():
    """对面发 /base_footprint → /base_link 时必须仍认得出，否则 auto 模式会去抢发。"""
    assert is_target_link("/base_footprint", "/base_link", "base_footprint", "base_link") is True
    assert is_target_link(" base_footprint ", "base_link/", "base_footprint", "base_link") is True
    assert is_target_link("/odom", "/base_link", "base_footprint", "base_link") is False


def test_normalize_frame_strips_slashes_and_blanks():
    assert normalize_frame("/base_link") == "base_link"
    assert normalize_frame("base_link/") == "base_link"
    assert normalize_frame("  base_link  ") == "base_link"
    assert normalize_frame(None) == ""
    assert normalize_frame("") == ""


def test_publish_due_throttles_resends():
    assert publish_due(None, 10_000_000_000, 1.0) is True
    assert publish_due(9_500_000_000, 10_000_000_000, 1.0) is False   # 才过 0.5s
    assert publish_due(9_000_000_000, 10_000_000_000, 1.0) is True    # 到 1s 了
    assert publish_due(9_950_000_000, 10_000_000_000, 0.05) is True
