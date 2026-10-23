"""巡检调度状态机（纯逻辑，不依赖 ROS，可直接单测）。

把"下一个该去哪个点"从节点里抽出来，原因：
  - 原实现把圈数/点位下标分散在节点回调里，return_home 参数被读取但从未生效，
    低电量返航后还会继续把剩下的点位跑完；
  - 抽成纯逻辑后，离线闭环演示的验收器（tools/offline_demo/demo_verify.py）
    与线上节点共用同一份调度规则，避免"验收脚本自己实现一套、和线上不一致"。

阶段 phase：
  patrol       巡检中，按点位顺序走
  return_home  返航阶段（走完最后一圈按要求回 home，或低电量被迫返航）
  done         无事可做（节点据此收工）

约定：
  - loops > 0  巡检圈数；loops <= 0 表示无限循环（永不停机）
  - return_home=True 时，走完最后一圈后再去一次 home 点位
  - home 点位 = 第一个 home: true 的点位；都没有则用第一个点位
"""
from __future__ import annotations

from typing import List, Optional, Sequence

from go2_patrol_patrol.waypoints import Waypoint

PHASE_PATROL = "patrol"
PHASE_RETURN_HOME = "return_home"
PHASE_DONE = "done"

REASON_FINISHED = "finished"
REASON_LOW_BATTERY = "low_battery"


class PatrolSequence:
    """按点位顺序巡圈，支持圈数、收工返航与低电量返航。"""

    def __init__(
        self,
        waypoints: Sequence[Waypoint],
        loops: int = 1,
        return_home: bool = True,
        home_index: Optional[int] = None,
    ):
        self.waypoints: List[Waypoint] = list(waypoints or [])
        self.loops = int(loops)
        self.return_home = bool(return_home)
        self.home_index = self._resolve_home_index(home_index)
        self.loop_index = 0
        self.waypoint_index = 0
        self.return_reason = ""
        self.phase = PHASE_DONE if not self.waypoints else PHASE_PATROL

    # ---------------------------------------------------------------- 初始化
    def _resolve_home_index(self, home_index: Optional[int]) -> Optional[int]:
        if not self.waypoints:
            return None
        if home_index is not None and 0 <= int(home_index) < len(self.waypoints):
            return int(home_index)
        for index, waypoint in enumerate(self.waypoints):
            if waypoint.home:
                return index
        return 0

    # ---------------------------------------------------------------- 查询
    @property
    def home(self) -> Optional[Waypoint]:
        if self.home_index is None:
            return None
        return self.waypoints[self.home_index]

    @property
    def done(self) -> bool:
        return self.phase == PHASE_DONE

    @property
    def returning(self) -> bool:
        return self.phase == PHASE_RETURN_HOME

    def describe(self) -> str:
        if self.done:
            return "done"
        total = "inf" if self.loops <= 0 else str(self.loops)
        if self.returning:
            return f"return_home({self.return_reason}) 圈 {self.loop_index}/{total}"
        return (
            f"patrol 圈 {self.loop_index + 1}/{total} "
            f"点位 {self.waypoint_index + 1}/{len(self.waypoints)}"
        )

    # ---------------------------------------------------------------- 调度
    def next_goal(self) -> Optional[Waypoint]:
        """返回当前该去的点位；None 表示收工（不要再发目标）。"""
        if self.phase == PHASE_DONE:
            return None
        if self.phase == PHASE_RETURN_HOME:
            return self.home
        return self.waypoints[self.waypoint_index]

    def following_waypoint(self) -> Optional[Waypoint]:
        """当前目标之后要去的点位；用于把到点朝向对齐"下一段"（yaw_mode: next）。

        没有后续点位（收工/返航，或本就只有一个点）时返回 None。
        """
        if self.phase != PHASE_PATROL or not self.waypoints:
            return None
        if self.waypoint_index + 1 < len(self.waypoints):
            return self.waypoints[self.waypoint_index + 1]
        if self.loops <= 0 or self.loop_index + 1 < self.loops:
            return self.waypoints[0]
        return self.home if (self.return_home and self.home is not None) else None

    def on_arrival(self, waypoint: Optional[Waypoint] = None) -> None:
        """到达当前目标后推进状态。"""
        if self.phase == PHASE_DONE:
            return
        if self.phase == PHASE_RETURN_HOME:
            self.phase = PHASE_DONE
            return

        arrived_home = self._is_home(waypoint)
        self.waypoint_index += 1
        if self.waypoint_index < len(self.waypoints):
            return

        self.waypoint_index = 0
        self.loop_index += 1
        if self.loops > 0 and self.loop_index >= self.loops:
            # 最后一圈走完：要么按配置回 home，要么收工。
            # 刚刚就在 home 点，就没必要再发一次同样的目标。
            if self.return_home and not arrived_home:
                self.phase = PHASE_RETURN_HOME
                self.return_reason = REASON_FINISHED
            else:
                self.phase = PHASE_DONE

    def request_return_home(self, reason: str = REASON_LOW_BATTERY) -> bool:
        """外部事件（低电量/急停）要求立刻返航；重复调用只生效一次。"""
        if self.phase in (PHASE_DONE, PHASE_RETURN_HOME):
            return False
        self.phase = PHASE_RETURN_HOME
        self.return_reason = reason
        return True

    def _is_home(self, waypoint: Optional[Waypoint]) -> bool:
        if waypoint is None or self.home is None:
            return False
        return waypoint.name == self.home.name

    # ---------------------------------------------------------------- 复制
    def copy(self) -> "PatrolSequence":
        """复制当前状态（验收器要在不影响线上调度的情况下预演一遍）。"""
        clone = PatrolSequence(self.waypoints, self.loops, self.return_home, self.home_index)
        clone.loop_index = self.loop_index
        clone.waypoint_index = self.waypoint_index
        clone.phase = self.phase
        clone.return_reason = self.return_reason
        return clone
