# AI 레이스크래프트 벤치마크

상태: **Bahrain v2 결정·이유·피트 우선권 게이트 구현 완료**
현재 런타임 기준: `planar_2d`

## 목적

AI가 추월 성공 여부를 직접 선택하는 대신, 현재 공간·속도·웨이크·DRS·남은 구간을
근거로 `attack`, `hold`, `abort`를 선택하고 물리가 결과를 확정하는지 반복 검증한다.
각 판단은 결과뿐 아니라 동일한 물리 게이트가 반환한 `reason_code`를 함께 기록한다.

## Bahrain v2 시나리오

| 시나리오 | 기대 판단 | 기대 이유 |
|---|---|---|
| 메인 스트레이트 | attack | `straight_window_open` |
| T1 근접 제동 | attack | `heavy_braking_window_open` |
| 중간 테크니컬 구간 | hold | `segment_disallows_attack` |
| T10 먼 제동 공격 | hold | `braking_gap_too_large` |
| T11–T12 다음 직선 준비 | hold | `segment_disallows_attack` |
| T13–T14 직선 | hold | `insufficient_closing_distance` — 현재 간격·접근속도로 차체를 안전하게 완전히 지울 거리가 부족 |
| 최종 제동 구간 | attack | `heavy_braking_window_open` |
| 피트 출구와 본선 그룹 | yield → hold → merge | 본선 그룹 우선권 |
| T1 racing line 점유 | outside | 안쪽 고정값이 아닌 280m 물리 시간·clearance 우선 |
| 코너 제동 preview 진입 | line committed | 제동 뒤 두 번째 방향 변경 없음 |

판단 레코드에는 범퍼 간격, 남은 거리, 사용 가능한 폭, 필요한 차폭, 상대 속도,
예상 접근속도, 추월 완료에 필요한 거리, 웨이크, DRS, 공격·실수 확률을 포함한다.

## 실행

```bash
cd backend
.venv/bin/python tools/run_bahrain_racecraft_benchmark.py --seeds 10 --seconds 2
```

기준 실행은 7개 시나리오 × 10 seed = 70회에서 판단 일치율과 이유 일치율 `100%`,
접촉 `0`, 트랙 이탈 `0`, 피트 `yield → hold → merge` 일치율 `100%`를 기록했다.
요약은 `backend/data/calibration/bahrain_racecraft_benchmark_v2.json`에 보존한다.

## 현재 한계와 다음 게이트

v1은 고정된 물리 상황에서 판단이 설명 가능하고 안전한지 검증한다. 실제 F1 리플레이의
공격 빈도·성공률·철회 시점과 통계적으로 일치한다고 주장하지 않는다. 다음 버전에서는
실제 영상으로 코너별 판단 라벨을 만들고, 50개 이상 seed의 완전한 구간 주행에서
공격 도달률, overlap, clear, abort, 접촉과 forced-wide 비율을 비교한다.

추가 회귀는 물리 계산 뒤 선행 차량 속도를 추종 차량에 복사하지 않는지, 실제 차체
배치에서 가장 가까운 선행 차량을 선택하는지, Bahrain T1의 바깥 corridor가 상대
점유 상황에서 실제 선택 가능한지를 고정 검증한다.
