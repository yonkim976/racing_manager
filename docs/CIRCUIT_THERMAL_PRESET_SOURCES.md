# 회로 열환경 프리셋 근거표

이 파일의 `COOL/NORMAL/HOT` 값은 연도별 예보나 특정 레이스의 공식 측정값이
아니다. 회로의 기후·경기 특성을 구분해 재현 가능한 정적 시뮬레이션 시나리오로
사용하기 위한 1차 경계조건이다. 따라서 아래 값은 후속 환경 보정 단계에서 공식
세션 기록, Pirelli 자료 또는 공인 기상자료와 대조할 때까지 `provisional`로
취급한다.

회로 식별과 경기 맥락은 Formula 1 공식 회로 페이지를 사용했다. 해당 페이지는
회로 정보와 공식 경기 일정을 제공하지만, 이 표의 세 프리셋 온도를 직접
제공하는 자료는 아니다.

| circuit_id / 서킷 | 프리셋 | 대기 °C | 노면 °C | 근거 종류 | 출처 URL | 상태 | 비고 |
|---|---|---:|---:|---|---|---|---|
| 3 / Bahrain International Circuit | COOL | 24 | 32 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Bahrain](https://www.formula1.com/en/racing/2025/bahrain) | provisional | Normal 모델 기준과 비교하기 위한 냉각 시나리오 |
| 3 / Bahrain International Circuit | NORMAL | 30 | 40 | 기존 장기 회귀 기준·정적 시나리오 | [Formula 1 Bahrain](https://www.formula1.com/en/racing/2025/bahrain) | provisional | 실측값이 아닌 1차 모델 회귀 기준선 |
| 3 / Bahrain International Circuit | HOT | 36 | 52 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Bahrain](https://www.formula1.com/en/racing/2025/bahrain) | provisional | 사막·고온 시나리오 |
| 4 / Red Bull Ring | COOL | 10 | 18 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Austria](https://www.formula1.com/en/racing/2025/austria) | provisional | 고지대 유럽 냉각 시나리오 |
| 4 / Red Bull Ring | NORMAL | 18 | 30 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Austria](https://www.formula1.com/en/racing/2025/austria) | provisional | 유럽 여름 기준 시나리오 |
| 4 / Red Bull Ring | HOT | 28 | 48 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Austria](https://www.formula1.com/en/racing/2025/austria) | provisional | 고온 시나리오 |
| 5 / Silverstone Circuit | COOL | 12 | 20 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Great Britain](https://www.formula1.com/en/racing/2025/great-britain) | provisional | 저온·저노면온도 시나리오 |
| 5 / Silverstone Circuit | NORMAL | 20 | 32 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Great Britain](https://www.formula1.com/en/racing/2025/great-britain) | provisional | 영국 여름 기준 시나리오 |
| 5 / Silverstone Circuit | HOT | 28 | 45 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Great Britain](https://www.formula1.com/en/racing/2025/great-britain) | provisional | 이례적 고온 시나리오 |
| 6 / Circuit de Spa-Francorchamps | COOL | 10 | 18 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Belgium](https://www.formula1.com/en/racing/2025/belgium) | provisional | 고지대·저온 시나리오 |
| 6 / Circuit de Spa-Francorchamps | NORMAL | 18 | 30 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Belgium](https://www.formula1.com/en/racing/2025/belgium) | provisional | 벨기에 여름 기준 시나리오 |
| 6 / Circuit de Spa-Francorchamps | HOT | 27 | 46 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Belgium](https://www.formula1.com/en/racing/2025/belgium) | provisional | 고온 시나리오 |
| 7 / Hungaroring | COOL | 18 | 28 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Hungary](https://www.formula1.com/en/racing/2025/hungary) | provisional | 일교차가 큰 냉각 시나리오 |
| 7 / Hungaroring | NORMAL | 27 | 43 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Hungary](https://www.formula1.com/en/racing/2025/hungary) | provisional | 여름 고온 기준 시나리오 |
| 7 / Hungaroring | HOT | 34 | 58 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Hungary](https://www.formula1.com/en/racing/2025/hungary) | provisional | 극단 고온 시나리오 |
| 8 / Autodromo Nazionale Monza | COOL | 16 | 26 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Italy](https://www.formula1.com/en/racing/2025/italy) | provisional | 저온 시나리오 |
| 8 / Autodromo Nazionale Monza | NORMAL | 24 | 38 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Italy](https://www.formula1.com/en/racing/2025/italy) | provisional | 이탈리아 여름 기준 시나리오 |
| 8 / Autodromo Nazionale Monza | HOT | 32 | 54 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Italy](https://www.formula1.com/en/racing/2025/italy) | provisional | 고온 시나리오 |
| 9 / Circuit Zandvoort | COOL | 13 | 22 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Netherlands](https://www.formula1.com/en/racing/2025/netherlands) | provisional | 해안·저온 시나리오 |
| 9 / Circuit Zandvoort | NORMAL | 20 | 32 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Netherlands](https://www.formula1.com/en/racing/2025/netherlands) | provisional | 네덜란드 여름 기준 시나리오 |
| 9 / Circuit Zandvoort | HOT | 27 | 45 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Netherlands](https://www.formula1.com/en/racing/2025/netherlands) | provisional | 이례적 고온 시나리오 |
| 10 / Circuit de Barcelona-Catalunya | COOL | 16 | 26 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Spain](https://www.formula1.com/en/racing/2025/spain) | provisional | 저온 시나리오 |
| 10 / Circuit de Barcelona-Catalunya | NORMAL | 24 | 38 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Spain](https://www.formula1.com/en/racing/2025/spain) | provisional | 지중해 여름 기준 시나리오 |
| 10 / Circuit de Barcelona-Catalunya | HOT | 32 | 54 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Spain](https://www.formula1.com/en/racing/2025/spain) | provisional | 고온 시나리오 |
| 11 / Suzuka International Racing Course | COOL | 15 | 24 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Japan](https://www.formula1.com/en/racing/2025/japan) | provisional | 봄철 냉각 시나리오 |
| 11 / Suzuka International Racing Course | NORMAL | 23 | 36 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Japan](https://www.formula1.com/en/racing/2025/japan) | provisional | 일본 봄철 기준 시나리오 |
| 11 / Suzuka International Racing Course | HOT | 30 | 50 | 공식 회로 맥락 + 보수적 시나리오 | [Formula 1 Japan](https://www.formula1.com/en/racing/2025/japan) | provisional | 습도·강수는 이번 단계에서 모델링하지 않음 |

다음 단계에서 각 회로의 실제 세션 대기·노면 측정값을 수집하면 이 표의
`provisional` 행을 별도 보정 기록으로 교체한다. 이 단계에서는 프리셋 간 순서와
바레인 Normal 회귀 기준을 우선 보장한다.
