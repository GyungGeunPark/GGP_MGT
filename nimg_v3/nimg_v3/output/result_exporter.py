"""
result_exporter.py - 결과 내보내기

측정 결과를 CSV, JSON 등 다양한 형식으로 내보냅니다.

Version: 1.0
Author: FurSys AI Team
"""

import csv
import json
import numpy as np
from pathlib import Path
from typing import List, Optional, Dict, Any
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


class ResultExporter:
    """
    측정 결과 내보내기

    CSV, JSON 형식으로 결과를 저장합니다.

    Example:
        >>> exporter = ResultExporter("output")
        >>> exporter.add_result(measurement_result)
        >>> exporter.save()
    """

    def __init__(
        self,
        output_dir: str,
        prefix: str = "nimg_v3",
        format: str = "csv"
    ):
        """
        Args:
            output_dir: 출력 디렉토리
            prefix: 파일 접두사
            format: 출력 형식 ("csv" 또는 "json")
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.prefix = prefix
        self.format = format

        self._results = []
        self._session_time = datetime.now().strftime("%Y%m%d_%H%M%S")

    def add_result(self, result: Any):
        """결과 추가"""
        if hasattr(result, 'to_dict'):
            self._results.append(result.to_dict())
        elif isinstance(result, dict):
            self._results.append(result)
        else:
            logger.warning(f"Unknown result type: {type(result)}")

    def add_results(self, results: List[Any]):
        """여러 결과 추가"""
        for r in results:
            if r is not None:
                self.add_result(r)

    def save(self, filename: Optional[str] = None) -> str:
        """
        결과 저장

        Returns:
            저장된 파일 경로
        """
        if not self._results:
            logger.warning("No results to save")
            return ""

        if filename is None:
            filename = f"{self.prefix}_{self._session_time}"

        if self.format == "csv":
            filepath = self._save_csv(filename)
        elif self.format == "json":
            filepath = self._save_json(filename)
        else:
            raise ValueError(f"Unknown format: {self.format}")

        logger.info(f"Results saved to {filepath}")
        return str(filepath)

    def _save_csv(self, filename: str) -> Path:
        """CSV로 저장"""
        filepath = self.output_dir / f"{filename}.csv"

        # 결과를 평탄화
        flat_results = []
        for result in self._results:
            flat = self._flatten_dict(result)
            flat_results.append(flat)

        if not flat_results:
            return filepath

        # 모든 키 수집
        all_keys = set()
        for r in flat_results:
            all_keys.update(r.keys())
        all_keys = sorted(all_keys)

        # CSV 작성
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=all_keys)
            writer.writeheader()
            writer.writerows(flat_results)

        return filepath

    def _save_json(self, filename: str) -> Path:
        """JSON으로 저장"""
        filepath = self.output_dir / f"{filename}.json"

        with open(filepath, 'w') as f:
            json.dump(self._results, f, indent=2, default=self._json_serializer)

        return filepath

    def _flatten_dict(self, d: dict, parent_key: str = '', sep: str = '_') -> dict:
        """중첩 딕셔너리 평탄화"""
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            elif isinstance(v, (list, np.ndarray)):
                if len(v) <= 4:
                    for i, val in enumerate(v):
                        items.append((f"{new_key}_{i}", val))
                else:
                    items.append((new_key, str(v)))
            else:
                items.append((new_key, v))
        return dict(items)

    def _json_serializer(self, obj):
        """JSON 직렬화 헬퍼"""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.integer):
            return int(obj)
        raise TypeError(f"Object of type {type(obj)} is not JSON serializable")

    def clear(self):
        """결과 초기화"""
        self._results = []

    @property
    def num_results(self) -> int:
        return len(self._results)

    def get_summary(self) -> Dict[str, Any]:
        """결과 요약"""
        if not self._results:
            return {}

        # 통계 계산
        speeds = []
        confidences = []

        for r in self._results:
            if 'kalman' in r and 'speed' in r['kalman']:
                speeds.append(r['kalman']['speed'])
            if 'pose' in r and 'confidence' in r['pose']:
                confidences.append(r['pose']['confidence'])

        return {
            'num_frames': len(self._results),
            'speed': {
                'mean': float(np.mean(speeds)) if speeds else 0,
                'std': float(np.std(speeds)) if speeds else 0,
                'max': float(np.max(speeds)) if speeds else 0
            },
            'confidence': {
                'mean': float(np.mean(confidences)) if confidences else 0,
                'min': float(np.min(confidences)) if confidences else 0
            }
        }
