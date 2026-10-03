"""独立执行接纳分支；不修改历史v6候选与已冻结实验。"""
from __future__ import annotations
import numpy as np
from swarm.recovery.recoverability_admission import RecoverabilityAdmissionCoordinator


class FailedVehicleLoiterPilot:
    """v2开发控制变量：失效机故障前悬停，健康机仍由目标产生名义意图。"""
    def actions(self, *, positions, velocities, goals):
        return {i: np.zeros(2) if i==2 else np.clip(1.5*(goals[i]-positions[i][:2]),-1.5,1.5)
                for i in positions}


class FixedRouteAdmissionCoordinator(RecoverabilityAdmissionCoordinator):
    """共享固定绝对航点库，仅当前起点由实测状态连接。"""
    def __init__(self, config, *, waypoint_routes, geometry_only=False):
        super().__init__(config, geometry_only=geometry_only)
        self.waypoint_routes = [[np.asarray(p[:2], dtype=float) for p in r] for r in waypoint_routes]
        self.decision_state = None

    def _candidate_routes(self, start, goal, failed_trajectory):
        return [[start.copy(), *[p.copy() for p in route], goal.copy()] for route in self.waypoint_routes]

    def _decide_from_snapshots(self, snapshots):
        self.decision_state = {str(i): {'position':list(s.position),
                                      'velocity':list(s.velocity or (0.,0.,0.))}
                               for i,s in snapshots.items()}
        super()._decide_from_snapshots(snapshots)

    def summary(self):
        return {**super().summary(), 'decision_state':self.decision_state,
                'fixed_waypoint_routes':[[p.tolist() for p in r] for r in self.waypoint_routes]}
