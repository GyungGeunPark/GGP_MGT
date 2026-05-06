"""Lost detector — reinit 트리거 판단.

architecture §6.2 에 대응.
"""
from __future__ import annotations


class LostDetector:
    """
    Reinit 을 **보수적으로** 트리거한다. 잦은 reinit 은 uid churn 을 유발하고
    KF 를 리셋시켜 속도/각속도 추정을 망가뜨린다.

    규칙:
    - tracker_ok=False 그 자체만으로는 reinit 하지 않는다 (grace 기간 허용).
    - (tracker_fail_count >= tracker_fail_frames) AND 이후 score 도 낮으면 reinit.
    - score < threshold 가 score_frames 연속일 때 reinit.
    - 주기적 reinit 은 periodic 프레임마다 1회.
    """

    def __init__(self, score_threshold: float = 0.4, score_frames: int = 5,
                 periodic_reinit_frames: int = 300,
                 tracker_fail_frames: int = 5):
        self.score_threshold = score_threshold
        self.score_frames = score_frames
        self.periodic_reinit_frames = periodic_reinit_frames
        self.tracker_fail_frames = tracker_fail_frames
        self._low_score_count = 0
        self._tracker_fail_count = 0
        self._frame_count = 0

    def update(self, score: float, tracker_ok: bool) -> bool:
        """True → reinit 필요."""
        self._frame_count += 1
        if tracker_ok:
            self._tracker_fail_count = 0
        else:
            self._tracker_fail_count += 1
            if self._tracker_fail_count >= self.tracker_fail_frames:
                self._tracker_fail_count = 0
                return True
        if score < self.score_threshold:
            self._low_score_count += 1
        else:
            self._low_score_count = 0
        if self._low_score_count >= self.score_frames:
            self._low_score_count = 0
            return True
        if self._frame_count >= self.periodic_reinit_frames:
            self._frame_count = 0
            return True
        return False

    def reset(self):
        self._low_score_count = 0
        self._tracker_fail_count = 0
        self._frame_count = 0
