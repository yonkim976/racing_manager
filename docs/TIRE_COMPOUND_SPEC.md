# 타이어 컴파운드 사양과 남은 승인 항목

상태: **C1~C5 도메인·세션 적용 완료 · C4/C5 보정 진행 중**
기준일: **2026-08-03**
적용 모드: `FULL` 물리엔진

## 권위 계약

물리 컴파운드와 한 경기 주말의 표시 역할은 서로 다른 값이다.

```text
Physical compound: C1, C2, C3, C4, C5, INTER, WET
Weekend role:      HARD, MEDIUM, SOFT
```

세션 생성 시 회로 지명과 컴파운드 사양을 불변 snapshot으로 확정한다. 예선, 출발,
전략, 피트, 차량 물리, 열·마모, 진단과 UI는 모두 같은 resolver 결과를 사용한다.
`SOFT` 같은 역할 문자열을 물리 계수 조회에 직접 사용해서는 안 된다.

## 공개 사실과 게임 보정값

- Pirelli 건식 범위는 C1부터 C5이며 C1이 가장 단단하고 C5가 가장 부드럽다.
- 한 경기에는 세 종류가 지명되고 그 주말에서 Hard, Medium, Soft 역할을 가진다.
- 2025 Bahrain은 C1/C2/C3가 지명됐다.
- 정확한 최적 표면·코어 온도, 열용량, 냉각계수와 degradation 수식은 공개 공식값이
  아니므로 프로젝트 수치는 `game_calibration_provisional`로 취급한다.

출처:

- [Pirelli Formula 1 tyre range](https://www.pirelli.com/tires/en-us/motorsport/car/formula-1)
- [Pirelli 2026 compound range](https://press.pirelli.com/the-range-of-compounds-for-the-2026-season-has-been-set/)
- [Pirelli compound nomination overview](https://www.pirelli.com/global/en-ww/race/racingspot/formula-1/formula-1-for-dummies-the-choice-of-tyres-in-formula-1-53864/)
- [Formula 1 2025 Bahrain nomination](https://www.formula1.com/en/latest/article/what-tyres-will-the-teams-and-drivers-have-for-the-2025-bahrain-grand-prix.7gnHbNpm0OJQIjPqsWoyof)

## 현재 provisional 사양

| C 코드 | grip | degradation | cliff laps | lateral | traction | braking | optimal °C | window ±°C | hot 진단 °C |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| C1 | 0.970 | 0.010 | 44 | 0.985 | 0.985 | 0.990 | 105 | 16 | 136 |
| C2 | 0.990 | 0.014 | 31 | 1.000 | 1.000 | 1.000 | 100 | 14 | 129 |
| C3 | 1.005 | 0.022 | 16 | 1.025 | 1.025 | 1.015 | 95 | 12 | 122 |
| C4 | 1.012 | 0.028 | 13 | 1.035 | 1.035 | 1.022 | 92 | 11 | 118 |
| C5 | 1.020 | 0.035 | 10 | 1.045 | 1.045 | 1.030 | 90 | 10 | 115 |

수치 권위는 코드의 컴파운드 사양이다. 이 표와 코드가 달라지면 의도된 보정인지
확인하고 문서를 함께 갱신한다. `160°C surface / 140°C core`는 성능 한계가 아니라
전역 수치 guard이며, 컴파운드별 작동창·hot 진단과 구분한다.

## 현재 구현과 남은 게이트

- C1~C5와 Hard/Medium/Soft 역할 분리, 회로별 지명, API, 예선, 전략, 피트, UI,
  진단 migration은 완료됐다.
- Bahrain은 `HARD=C1`, `MEDIUM=C2`, `SOFT=C3`이며 NORMAL 57랩·20대 완주가 승인됐다.
- C4/C5는 초기 seed이므로 대표 회로에서 단축 제품 검증과 장기 열·마모 보정이 남아 있다.
- 모든 대표 실행은 `C5 > C4 > C3 > C2 > C1`의 fresh pace와 반대 방향의 내구성,
  작동창 반응, clamp 0, New Race cleanup을 함께 확인해야 한다.
- 출발 90°C와 피트 70°C 분리는 별도
  [`COLD_OUTLAP_CONTROL_DESIGN.md`](COLD_OUTLAP_CONTROL_DESIGN.md) 작업이다.
