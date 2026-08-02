# FULL 모드 Bahrain·Red Bull Ring 회로 보정 상태

상태: **교통 안전 승인 · 엄격한 기본 레이싱라인 승인 보류**
기준일: **2026-08-03**
적용 범위: `FULL`, 레이스 연료, NORMAL, 주말 Medium, STANDARD, 0.02초/50Hz

## 현재 결정

- 전역 차량 모델의 load-sensitive cornering stiffness와 paired target/reference line
  transport는 회귀를 통과해 채택했다.
- Bahrain과 Red Bull Ring 모두 20대 10랩 교통 안전성은 승인됐다.
- 두 회로의 급격한 기준선 전환을 제한하는 lateral slope와 RBR 차체 edge buffer는
  교통 안전 보정으로 채택했다. 다만 엄격한 단독 레이싱라인 오차 기준은 통과하지
  못했으므로 회로 전체를 `calibrated`로 최종 승인하지 않는다.
- 퀄리파잉 라인은 레이스 기본선 승인 뒤 별도 단계로 진행한다.

## 데이터와 진단 권위

- 실제 회로 형상·출처·라이선스는
  [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md)를 따른다.
- 재현 도구는 `backend/tools/run_circuit_baseline.py`와
  `backend/tools/run_race_line_traffic.py`다.
- 기준 산출물은 `backend/data/calibration/`에 저장한다.
- 각 표본은 compiled-centerline progress, active path progress, telemetry progress,
  centerline/line-relative offset, curvature, lateral speed와 안전 counter를 구분한다.

## 단독 기준선과 채택된 기반 보정

초기 5랩×3회 기준은 다음과 같았다.

| 회로 | fastest lap | lateral p95/max | heading p95 | >2m 시간 | 안전 counter |
|---|---:|---:|---:|---:|---|
| Bahrain | 92.907s | 4.785/6.141m | 0.105rad | 53.46s | 모두 0 |
| Red Bull Ring | 66.990s | 5.759/7.918m | 0.105rad | 54.40s | 모두 0 |

reference smoothing, lateral slope, feed-forward, response cap, 전역 grip·속도·yaw 후보는
목표 미달, fallback, off-track 위험 또는 과도한 랩 손실 때문에 채택하지 않았다.

55m/s·곡률 0.014/m fixture에서 고다운포스 횡력 여유와 별개로 cornering stiffness가
정적 하중에 고정돼 heading guard를 소모하는 원인을 확인했다. 전·후축 stiffness를
수직하중 용량 비의 0.65승, 최대 2.5배로 보정하되 전체 횡력 상한은 바꾸지 않았다.

보정 후 5랩×3회 full-race-fuel 결과:

| 회로 | lap | lateral p95/max | heading-limit 시간 | fallback | rear surface/core peak |
|---|---:|---:|---:|---:|---:|
| Bahrain | 94.149s | 3.733/4.883m | 61.66s | 0 | 107.917/94.423°C |
| Red Bull Ring | 67.997s | 5.488/7.582m | 53.60s | 250/run | 103.170/91.666°C |

승인 기준은 lateral p95 `≤2m`, max `≤4m`, heading p95 `≤0.08rad`, 안전 counter와
clean-car planner fallback 0이다. 따라서 전역 차량 보정만 승인하고 회로별 기본선은
보류한다. 주요 잔여 구간은 Bahrain T4–T6·T11–T12, RBR T6–T7·T9–T10이다.

## 20대 교통 승인

제품 스타트, 20대, 10랩, 50Hz 결정론 실행 결과:

| 항목 | Bahrain | Red Bull Ring |
|---|---:|---:|
| 완주/은퇴 | 20/0 | 20/0 |
| contact/off-track/track-limit | 0/0/0 | 0/0/0 |
| planner fallback | 0 | 0 |
| traffic hold samples | 145 | 169 |
| 최대 횡이동/step | 0.08m | 0.08m |
| 최대 tactical rejoin | 17.12s | 14.94s |
| rear surface/core peak | 111.137/98.782°C | 99.985/92.214°C |
| 과열/thermal clamp | 0/0 | 0/0 |

안전한 로컬 후보가 없을 때 합성 tactical line을 만들지 않는 `traffic_hold`, 물리적
차로 수렴을 고려한 앞차 검색, 제3차량 corridor 예약, yaw 반영 차체 간격과 첫 코너
뒤 그리드 합류가 이 승인에 포함된다.

## 남은 FULL 모드 작업

1. 작은 결정론 fixture로 active-line pose/path 정렬과 heading saturation을 분리한다.
2. RBR 측정 기준선의 급격한 T6–T7·T9–T10 전환을 데이터 문제와 제어 문제로 나눈다.
3. 단독 5랩 오차 기준을 통과한 뒤 15랩 stint와 20대 교통을 다시 승인한다.
4. 레이스 기본선 승인 전에는 퀄리파잉 전용 공격성·Soft·연석 사용을 섞지 않는다.
5. `ABSTRACT` 실험을 이유로 이 `FULL` 회귀 기준을 완화하거나 삭제하지 않는다.
