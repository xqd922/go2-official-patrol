from go2_patrol_patrol.speech_policy import (
    DROP,
    PLAY,
    QUEUE,
    decide,
    pending_is_fresh,
)


def test_plays_when_idle():
    assert decide("home", playing=None, busy=False, cooldown_active=False) == PLAY


def test_queues_while_another_speech_is_playing():
    """转弯提醒正在播时来了到点语音：必须排队，不能静默丢掉。"""
    assert decide("equipment_zone", playing="turn_careful", busy=True, cooldown_active=False) == QUEUE


def test_queues_during_cooldown():
    assert decide("zone_ne", playing=None, busy=False, cooldown_active=True) == QUEUE


def test_drops_empty_and_already_playing():
    assert decide("", playing=None, busy=False, cooldown_active=False) == DROP
    assert decide("   ", playing=None, busy=False, cooldown_active=False) == DROP
    assert decide("home", playing="home", busy=True, cooldown_active=False) == DROP


def test_pending_is_fresh_has_a_ttl():
    assert pending_is_fresh(1_000_000_000, 2_000_000_000, 2.0) is True
    assert pending_is_fresh(1_000_000_000, 3_000_000_000, 2.0) is True
    assert pending_is_fresh(1_000_000_000, 4_500_000_000, 2.0) is False


def test_pending_without_timestamp_is_never_fresh():
    assert pending_is_fresh(None, 1_000_000_000, 2.0) is False
