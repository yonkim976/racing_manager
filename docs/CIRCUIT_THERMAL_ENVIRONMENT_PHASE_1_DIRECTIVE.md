# 서킷 열환경 기반 1차 작업 지시서

## 0. 결정 사항

이번 1차 작업은 **C1~C5 컴파운드 물성을 추가하기 전에 서킷별 정적 열환경을 확정하고, 그 환경을 세션 전체의 단일 기준값으로 전달하는 작업**이다.

구현 순서는 다음과 같이 고정한다.

```text
서킷별 Cool/Normal/Hot 데이터
→ API 및 Race Setup 선택
→ 세션 생성 시 1회 해석
→ 타이어·브레이크·텔레메트리에 동일 값 전달
→ 환경 민감도와 기존 Normal 회귀 검증
→ 이후 별도 작업에서 C1~C5 적용
```

이 순서를 지키는 이유는 컴파운드와 환경을 동시에 바꾸면 온도 변화가 타이어 물성 때문인지 대기·노면 온도 때문인지 분리하기 어렵기 때문이다.

별도 요청이 없으면 커밋과 푸시는 하지 않는다.

---

## 1. 목표

1차 작업이 끝나면 다음 조건을 만족해야 한다.

1. 현재 등록된 모든 서킷이 `COOL`, `NORMAL`, `HOT` 세 가지 정적 열환경 프리셋을 가진다.
2. Race Setup에서 프리셋과 실제 대기·노면 온도를 확인하고 선택할 수 있다.
3. 선택값은 레이스 세션 생성 시 한 번만 해석되고 세션 동안 불변이다.
4. 타이어 열 모델과 브레이크 열 모델이 동일한 세션 대기 온도를 사용한다.
5. API 응답, WebSocket 초기 정보, 틱 상태, 데스크톱 진단이 실제 해석값을 동일하게 표시한다.
6. 바레인 `NORMAL`은 현재 기준인 대기 `30°C`, 노면 `40°C`를 유지한다.
7. 같은 물리 입력에서 `COOL < NORMAL < HOT` 순으로 장기 평형 온도가 높아진다.
8. 기존 바레인 Normal 타이어 회귀 결과가 환경 계약 변경만으로 크게 달라지지 않는다.

---

## 2. 이번 작업에서 하지 않는 것

다음 항목은 1차 범위에 포함하지 않는다.

- C1~C5 타이어 물성 추가
- 주말별 `SOFT/MEDIUM/HARD`와 C1~C5 매핑
- 타이어 최적 온도창, 마모율, 그립, 열용량 또는 열계수 보정
- 브레이크 열용량, 발열률, 냉각률, 최적 온도 또는 페이드 기준 보정
- 비, 수막, 습도, 바람, 일사량, 구름
- 시간대에 따른 노면 온도 변화
- 랩 진행에 따른 온도 변화
- 무작위 날씨 생성
- 피트스톱 또는 세이프티카 진입 시 환경값 변경
- 환경에 따른 랩타임·엔진출력·공력 성능 보정
- 현재 `SOFT/MEDIUM/HARD` API 의미 변경

이번 단계는 **경계조건과 전달 구조**를 만드는 작업이다. 물리 계수 보정은 후속 단계에서 독립적으로 수행한다.

---

## 3. 현재 코드에서 반드시 주의할 점

### 3.1 `Circuit.track_conditions`는 기온 데이터가 아니다

현재 `/Users/kimyongjin/Desktop/f1/backend/models/schemas.py`의 `Circuit.track_conditions`는 `TrackConditionSample` 목록이다. 이 필드는 트랙 진행률별 고도, 경사, 뱅킹, 노면 종류를 담는다.

따라서 열환경 필드에 다음 이름을 사용하면 안 된다.

```text
Circuit.track_conditions
```

열환경에는 별도의 이름을 사용한다.

```text
Circuit.thermal_profile
```

세션에서 최종 해석된 단일 환경값은 기존 의미대로 다음 이름을 유지한다.

```text
RaceEngine.track_conditions: TrackConditions
```

### 3.2 현재 Race Setup은 환경값을 보내지 않는다

`/Users/kimyongjin/Desktop/f1/frontend/src/components/RaceSetup.jsx`는 현재 레이스 설정 요청에 서킷, 팀, 랩 수, 시작 타이어와 그리드만 보낸다. 그 결과 백엔드의 전역 기본값 `30/40°C`가 모든 서킷에 사용된다.

### 3.3 브레이크 외기 온도는 30°C로 고정돼 있다

`/Users/kimyongjin/Desktop/f1/backend/simulation/brake_model.py`는 `BRAKE_AMBIENT_TEMPERATURE_C = 30.0`을 냉각 기준으로 사용한다. 타이어만 세션 환경을 사용하고 브레이크는 고정값을 사용하면 동일 세션에서 서로 다른 날씨가 적용된다.

### 3.4 세션 환경 자체는 이미 불변 객체다

기존 `TrackConditions`는 Pydantic frozen model이다. 이 성질을 유지하고, 프리셋 해석은 세션 생성 경계에서만 수행한다.

---

## 4. 목표 데이터 구조

이름은 프로젝트 스타일에 맞게 소폭 조정할 수 있지만 의미와 불변식은 유지한다.

### 4.1 프리셋 이름

```python
class ThermalPresetName(str, Enum):
    COOL = "COOL"
    NORMAL = "NORMAL"
    HOT = "HOT"
```

대소문자 계약은 프런트엔드와 백엔드에서 하나로 통일한다. 기존 enum 스타일에 맞춰 대문자 값을 권장한다.

### 4.2 서킷 열환경 프로필

```python
class CircuitThermalProfile(BaseModel):
    default_preset: ThermalPresetName = ThermalPresetName.NORMAL
    presets: dict[ThermalPresetName, TrackConditions]
```

필수 불변식은 다음과 같다.

- `COOL`, `NORMAL`, `HOT`가 정확히 한 개씩 존재한다.
- `default_preset`이 `presets`에 존재한다.
- 각 온도는 유한값이며 기존 `TrackConditions` 허용 범위 안이다.
- 대기 온도는 `COOL <= NORMAL <= HOT`이다.
- 노면 온도도 `COOL <= NORMAL <= HOT`이다.
- `track_temperature >= ambient_temperature`를 전역 규칙으로 강제하지 않는다. 야간 또는 급격한 냉각 조건에서는 항상 참이라고 볼 수 없기 때문이다.

`Circuit`에는 다음 필드를 새로 추가한다.

```python
thermal_profile: CircuitThermalProfile
```

### 4.3 원본 데이터 파일

거대한 트랙 형상 데이터와 기후 기준값을 섞지 않도록 다음 별도 파일을 권장한다.

```text
/Users/kimyongjin/Desktop/f1/backend/data/circuit_thermal_profiles.json
```

권장 형태:

```json
[
  {
    "circuit_id": 3,
    "default_preset": "NORMAL",
    "presets": {
      "COOL": {
        "ambient_temperature_c": 24.0,
        "track_temperature_c": 32.0
      },
      "NORMAL": {
        "ambient_temperature_c": 30.0,
        "track_temperature_c": 40.0
      },
      "HOT": {
        "ambient_temperature_c": 36.0,
        "track_temperature_c": 52.0
      }
    }
  }
]
```

바레인은 위 값을 1차 고정값으로 사용한다. `NORMAL 30/40°C`는 기존 장기 열 회귀의 기준을 보존하기 위한 값이다. `COOL 24/32°C`, `HOT 36/52°C`는 현재 열 모델 환경 민감도 테스트에서 이미 사용되는 시나리오와 맞춘다.

나머지 서킷 값은 임의로 “실제 공식값”이라 주장하지 않는다. 구현 전에 근거표를 작성하고, 정적 시뮬레이션 시나리오라는 점을 명시한다.

```text
/Users/kimyongjin/Desktop/f1/docs/CIRCUIT_THERMAL_PRESET_SOURCES.md
```

근거표에는 최소한 다음 열이 있어야 한다.

| 항목 | 설명 |
|---|---|
| circuit_id / 서킷 | 런타임 데이터와 연결할 키 |
| 프리셋 | COOL / NORMAL / HOT |
| 대기 온도 | °C |
| 노면 온도 | °C |
| 근거 종류 | 공식 세션 기록, 타이어 공급사 자료, 기상자료, 보수적 추정 |
| 출처 URL | 직접 확인 가능한 페이지 |
| 상태 | calibrated 또는 provisional |
| 비고 | 야간 경기, 계절, 극단값 여부 |

출처 우선순위는 다음과 같다.

1. FIA, Formula 1, Pirelli 등 공식 세션·이벤트 자료
2. 서킷 또는 공인 기상기관 자료
3. 신뢰할 수 있는 모터스포츠 데이터
4. 자료가 부족할 때만 보수적 추정

`COOL/NORMAL/HOT`는 매년의 정확한 예보가 아니라 테스트 가능한 정적 시나리오다. 특정 연도의 한 세션 값을 그대로 모든 해의 절대 기준으로 취급하지 않는다.

---

## 5. 데이터 로딩 계약

`load_circuits()`에서 트랙 데이터와 열환경 프로필을 `circuit_id`로 결합한다.

앱 시작 시 다음 오류를 즉시 검출해야 한다.

- 현재 `circuits.json`의 서킷에 열환경 프로필이 없음
- 같은 `circuit_id`가 두 번 존재
- 존재하지 않는 서킷 ID의 프로필이 존재
- 세 프리셋 중 하나라도 없음
- 프리셋 온도 순서가 뒤집힘
- NaN, Infinity 또는 허용 범위 밖의 온도

조용히 전역 `30/40°C`를 대입하는 런타임 fallback은 만들지 않는다. 현재 등록된 9개 서킷 모두 명시적 프로필을 가져야 한다.

필요하면 테스트 fixture만을 위한 프로필 factory를 둔다. 프로덕션 데이터 누락을 숨기는 fallback과 테스트 편의용 factory를 혼동하지 않는다.

프로필 결합은 앱 시작 시 한 번 수행한다. 물리 틱마다 JSON을 읽거나 딕셔너리를 다시 구성하지 않는다.

---

## 6. 요청 및 세션 해석 계약

### 6.1 Race Setup 요청

기존 클라이언트와 테스트가 `track_conditions`를 직접 제공할 수 있도록 명시적 override 기능은 유지한다. 다만 기본값을 즉시 생성하지 말고 `None`으로 구분한다.

```python
class RaceSetupRequest(BaseModel):
    ...
    thermal_preset: ThermalPresetName | None = None
    track_conditions: TrackConditions | None = None
```

### 6.2 해석 우선순위

`SessionManager.create_session()`에서 다음 우선순위를 정확히 적용한다.

```text
1. request.track_conditions가 있으면 명시적 override
2. request.thermal_preset이 있으면 해당 서킷의 프리셋
3. 둘 다 없으면 해당 서킷의 default_preset
```

다음 모호한 요청은 `400`으로 거부한다.

```text
thermal_preset과 track_conditions를 동시에 제공
```

둘을 동시에 허용하고 한쪽을 조용히 무시하면 사용자가 다른 환경으로 테스트했다고 착각할 수 있다.

### 6.3 세션 소유 상태

세션은 다음 세 값을 소유한다.

```python
resolved_track_conditions: TrackConditions
resolved_thermal_preset: ThermalPresetName | None
track_conditions_source: Literal[
    "circuit_preset",
    "explicit_override",
]
```

명시적 override에는 대응 프리셋이 없으므로 `resolved_thermal_preset=None`이어도 된다.

프리셋의 딕셔너리 또는 `CircuitThermalProfile` 전체를 물리 루프에서 계속 조회하지 않는다. `RaceEngine`에는 최종 `TrackConditions` 객체 하나를 전달한다.

### 6.4 불변성

세션이 시작된 뒤에는 다음 이벤트로 환경값을 변경하지 않는다.

- 랩 전환
- 피트인·피트아웃
- SC/VSC 전환
- 일시정지·재개
- 1x/2x 변경
- WebSocket 재연결

환경 변경은 새 레이스 세션을 만들어야만 가능하다.

---

## 7. API와 화면 표시

### 7.1 `/api/circuits`

각 서킷 응답에 `thermal_profile`을 포함한다. 프런트엔드가 온도를 별도로 하드코딩하지 않도록 한다.

### 7.2 Race Setup UI

Race Setup에 다음 요소를 추가한다.

- `Cool`, `Normal`, `Hot` 세 선택지
- 선택한 프리셋의 대기 온도
- 선택한 프리셋의 노면 온도
- 현재 서킷의 기본 프리셋 표시

동작 규칙:

- 최초 로딩 시 첫 서킷의 `default_preset`을 선택한다.
- 서킷을 변경하면 새 서킷의 `default_preset`으로 재설정한다.
- 환경 프리셋을 변경하면 기존 예선 결과를 무효화한다. 향후 예선도 동일 환경 계약을 사용하게 될 때 오래된 그리드를 재사용하지 않기 위함이다.
- 레이스 요청에는 선택한 `thermal_preset`을 포함한다.
- 고급 사용자용 직접 온도 입력 UI는 이번 단계에서 만들지 않는다.
- API의 명시적 `track_conditions` override는 자동 테스트와 개발 도구용으로만 유지한다.

### 7.3 예선 요청

Race Setup에서 선택한 프리셋을 예선 요청에도 포함한다.

```python
class QualifyingRequest(BaseModel):
    ...
    thermal_preset: ThermalPresetName | None = None
```

이번 단계에서 환경이 예선 랩타임 물리에 영향을 주지 않더라도, 예선 응답이 사용한 프리셋과 해석된 온도를 반환해 주말 설정의 재현성을 유지한다. 환경으로 예선 성능을 보정하는 작업은 범위 밖이다.

### 7.4 응답 및 진단

최소한 다음 위치에서 같은 해석값을 확인할 수 있어야 한다.

- `RaceSetupResponse`
- `RaceInfoMessage.environment_conditions`
- `RaceTickState.track_conditions`
- `RaceEngine.diagnostic_counts()`
- 데스크톱 진단 로그

권장 응답 필드:

```text
thermal_preset
track_conditions_source
track_conditions:
  ambient_temperature_c
  track_temperature_c
```

프런트엔드 표시값이 API 요청값만 반영해서는 안 된다. 레이스 시작 후에는 백엔드 응답의 최종 해석값을 권위값으로 표시한다.

---

## 8. 타이어 및 브레이크 전달

### 8.1 타이어

타이어는 이미 `RaceEngine.track_conditions`를 통해 대기·노면 온도를 받는 경로가 있으므로 이 경로를 유지한다.

이번 단계에서 다음을 변경하지 않는다.

- 표면·코어 열용량
- 발열원별 변환계수
- 주행풍·노면·코어 냉각계수
- 표면·코어 상한
- 열 그립 곡선

### 8.2 브레이크

`advance_brake_thermal_state()`에 대기 온도를 필수 인자로 추가한다.

```python
def advance_brake_thermal_state(
    *,
    ...
    ambient_temperature_c: float,
) -> BrakeThermalState:
```

`strategy_ops.py`는 `self.track_conditions.ambient_temperature_c`를 전달한다.

브레이크 냉각식의 고정 기준:

```python
BRAKE_AMBIENT_TEMPERATURE_C = 30.0
```

은 제거하고 전달된 대기 온도를 사용한다.

이번 단계에서는 브레이크의 발열·냉각 계수를 바꾸지 않는다. 최소 온도와 최대 온도 상수도 별도의 모델 보정 근거 없이 수정하지 않는다.

모든 직접 호출 테스트는 대기 온도를 명시적으로 넘기게 수정한다. 함수 인자에 조용한 `30.0` 기본값을 두면 전달 누락을 검출할 수 없으므로 권장하지 않는다.

---

## 9. 구현 순서

### 9.1 데이터 계약

1. `ThermalPresetName`과 `CircuitThermalProfile`을 추가한다.
2. `Circuit.thermal_profile`을 추가한다.
3. 모델 validator로 세 프리셋 존재, 유한값, 온도 순서를 검증한다.
4. 기존 `Circuit.track_conditions`의 의미와 직렬화를 변경하지 않는다.

### 9.2 데이터와 로더

1. `circuit_thermal_profiles.json`을 추가한다.
2. 9개 현행 서킷의 프로필을 모두 작성한다.
3. `CIRCUIT_THERMAL_PRESET_SOURCES.md`에 출처와 provisional 여부를 기록한다.
4. `load_circuits()`에서 ID로 1회 결합한다.
5. 누락·중복·미등록 ID를 startup error로 처리한다.

### 9.3 백엔드 요청과 세션

1. Race Setup 요청에 preset과 optional override를 추가한다.
2. 상호 배타성을 검증한다.
3. 세션 생성 경계에서 최종 환경을 한 번 해석한다.
4. `RaceEngine`에는 해석된 불변 `TrackConditions`만 전달한다.
5. 응답과 진단에 preset/source/온도를 노출한다.

### 9.4 브레이크 환경 전달

1. 브레이크 함수에 필수 대기 온도 인자를 추가한다.
2. 모든 호출자가 세션 대기 온도를 전달하도록 한다.
3. 고정 30°C 냉각 기준을 제거한다.
4. 계수는 변경하지 않는다.

### 9.5 프런트엔드

1. 서킷 응답의 `thermal_profile`을 사용한다.
2. Cool/Normal/Hot 선택기와 온도 미리보기를 추가한다.
3. 서킷·환경 변경 시 예선 결과를 초기화한다.
4. 예선과 레이스 요청에 같은 프리셋을 전송한다.
5. 레이스 시작 후 백엔드의 해석 결과를 사용한다.

### 9.6 테스트와 문서

1. 데이터 무결성 테스트를 추가한다.
2. API·세션·프런트엔드 계약 테스트를 추가한다.
3. 타이어·브레이크 환경 민감도 테스트를 추가한다.
4. 바레인 Normal 장기 회귀를 실행한다.
5. 결과와 남은 provisional 데이터를 상태 문서에 기록한다.

---

## 10. 예상 수정 파일

구조상 필요한 파일만 수정한다. 다음 목록은 예상 범위다.

```text
backend/models/schemas.py
backend/data/circuit_thermal_profiles.json
backend/data_loader.py
backend/session.py
backend/main.py
backend/simulation/brake_model.py
backend/simulation/strategy_ops.py
backend/simulation/race_engine.py
backend/tests/test_api.py
backend/tests/test_session.py
backend/tests/test_fuel_and_tire_strategy.py
backend/tests/test_tire_thermal_foundation.py
backend/tests/test_circuit_thermal_profiles.py
frontend/src/components/RaceSetup.jsx
frontend/src/components/RaceSetup.css
frontend/test/*
docs/CIRCUIT_THERMAL_PRESET_SOURCES.md
docs/CURRENT_PROJECT_STATUS.md
```

실제 구현상 필요하지 않은 파일은 억지로 수정하지 않는다.

---

## 11. 필수 테스트

### 11.1 데이터 무결성

1. 현재 9개 서킷 모두 정확히 한 개의 열환경 프로필을 가진다.
2. 모든 프로필에 Cool/Normal/Hot가 존재한다.
3. 대기와 노면 온도의 순서가 각각 Cool ≤ Normal ≤ Hot다.
4. 중복 ID는 로딩 실패한다.
5. 누락 ID는 로딩 실패한다.
6. 존재하지 않는 서킷 ID는 로딩 실패한다.
7. NaN, Infinity, 범위 밖 값은 검증 실패한다.
8. 바레인 값은 `24/32`, `30/40`, `36/52°C`와 일치한다.
9. 기존 지형용 `Circuit.track_conditions`가 이전과 동일하게 로딩된다.

### 11.2 요청 해석

1. preset과 override가 모두 없으면 서킷 기본 프리셋이 선택된다.
2. `COOL`, `NORMAL`, `HOT` 요청이 정확한 값으로 해석된다.
3. 명시적 `track_conditions` override가 그대로 사용된다.
4. preset과 override를 함께 보내면 `400`이다.
5. 잘못된 preset은 `422` 또는 프로젝트 표준 validation error다.
6. 이전 클라이언트처럼 환경 필드를 생략한 요청도 해당 서킷 기본값으로 성공한다.

### 11.3 세션 불변성과 전파

1. 엔진, setup response, race info, tick, diagnostics의 온도가 일치한다.
2. 세션 시작 후 원본 프로필 객체를 바꾸려 해도 활성 세션 값은 변하지 않는다.
3. SC/VSC, 피트인·아웃, 1x/2x 전환으로 환경값이 변하지 않는다.
4. 새 세션을 만들면 이전 세션의 환경값이 남지 않는다.
5. `New Race` cleanup 후 환경 관련 캐시 또는 태스크가 남지 않는다.

### 11.4 타이어 환경 민감도

동일 차량, 동일 입력, 동일 seed, 동일 스텝 수에서 바레인 세 프리셋을 비교한다.

필수 불변식:

```text
cool rear surface equilibrium
< normal rear surface equilibrium
< hot rear surface equilibrium

cool rear core equilibrium
< normal rear core equilibrium
< hot rear core equilibrium
```

추가 조건:

- 각 실행의 입력 에너지 누계는 같은 허용오차 안에서 일치한다.
- 차이는 환경 냉각·노면 교환에서 발생해야 한다.
- 정상 시나리오에서 surface/core hard clamp는 0이다.
- 기존 장기 열수지 진단 필드가 유실되지 않는다.

### 11.5 브레이크 환경 민감도

초기 브레이크 온도, 속도, 제동력, 바이어스와 실행 시간을 동일하게 두고 대기 온도만 바꾼다.

고온 브레이크의 무제동 냉각에서:

```text
COOL 환경의 최종 브레이크 온도
< NORMAL 환경의 최종 브레이크 온도
< HOT 환경의 최종 브레이크 온도
```

같은 대기 온도 `30°C`를 명시했을 때 기존 브레이크 테스트 결과는 허용오차 안에서 유지되어야 한다.

### 11.6 프런트엔드

1. 선택한 서킷의 기본 프리셋이 표시된다.
2. preset 변경 시 대기·노면 미리보기가 갱신된다.
3. 서킷 변경 시 새 기본 프리셋으로 재설정된다.
4. 서킷 또는 preset 변경 시 기존 예선 결과가 제거된다.
5. 예선과 레이스 요청에 같은 preset이 포함된다.
6. 온도는 프런트엔드 상수가 아니라 `/api/circuits` 응답에서 읽는다.
7. 기존 팀, 랩 수, 시작 타이어, 그리드 선택 동작이 유지된다.

---

## 12. 검증 명령

프로젝트의 실제 테스트 구조에 맞춰 파일명은 조정할 수 있다.

```bash
cd /Users/kimyongjin/Desktop/f1/backend
.venv/bin/python -m pytest tests/test_circuit_thermal_profiles.py -q
.venv/bin/python -m pytest tests/test_session.py tests/test_api.py -q
.venv/bin/python -m pytest tests/test_tire_thermal_foundation.py -q
.venv/bin/python -m pytest tests/test_fuel_and_tire_strategy.py -q
.venv/bin/python -m pytest -q
```

```bash
cd /Users/kimyongjin/Desktop/f1/frontend
npm test
npm run lint
npm run build
```

```bash
cd /Users/kimyongjin/Desktop/f1
git diff --check
```

전체 테스트가 지나치게 오래 걸리면 관련 테스트를 먼저 완료하되, 최종 보고서에 전체 테스트의 실행 여부와 중단 지점을 정확히 기록한다.

---

## 13. 완료 승인 기준

다음을 모두 만족해야 1차 완료로 판정한다.

- [ ] 현행 9개 서킷의 열환경 프로필이 명시적으로 존재한다.
- [ ] 각 값의 출처 또는 provisional 근거가 문서화돼 있다.
- [ ] 기존 지형용 `Circuit.track_conditions`와 새 `thermal_profile`이 혼동되지 않는다.
- [ ] 환경 필드를 생략한 기존 Race Setup 요청이 서킷 기본값으로 동작한다.
- [ ] preset과 explicit override의 모호한 동시 요청이 거부된다.
- [ ] 세션 환경이 생성 시 한 번 해석되고 경기 중 불변이다.
- [ ] 타이어와 브레이크가 동일한 세션 대기 온도를 사용한다.
- [ ] UI에서 preset과 실제 두 온도를 확인할 수 있다.
- [ ] 바레인 Normal은 정확히 `30/40°C`다.
- [ ] Cool/Normal/Hot 타이어 평형 온도 순서가 검증된다.
- [ ] Cool/Normal/Hot 브레이크 냉각 순서가 검증된다.
- [ ] 바레인 Normal의 기존 장기 결과가 허용 범위 안에서 유지된다.
- [ ] 정상 시나리오에서 타이어 hard clamp가 발생하지 않는다.
- [ ] 관련 백엔드 테스트, 프런트엔드 테스트, lint, build가 통과한다.
- [ ] `git diff --check`가 통과한다.
- [ ] C1~C5 또는 타이어·브레이크 계수 보정이 섞이지 않았다.

### 바레인 Normal 회귀 허용 범위

환경 계약 변경 전후에 같은 seed, 차량, 입력, 스텝, 실행 시간을 사용한다.

- 입력 에너지 누계: 기존 대비 ±1% 이내
- 후륜 표면 최고 온도: 기존 대비 ±1°C 이내
- 후륜 코어 최고 온도: 기존 대비 ±1°C 이내
- 130°C 초과 차량 수와 clamp 횟수: 증가하지 않음

기존 결정론 테스트가 더 엄격한 허용오차를 사용한다면 더 엄격한 기존 기준을 유지한다.

---

## 14. 금지되는 우회 구현

- 모든 서킷에 같은 값을 복사하고 “서킷별 적용 완료”라고 보고하지 않는다.
- 데이터 누락 시 조용히 `30/40°C`로 fallback하지 않는다.
- 프런트엔드에 서킷 온도를 별도 하드코딩하지 않는다.
- `Circuit.track_conditions`를 열환경 용도로 재사용하지 않는다.
- 세션 도중 프리셋 객체를 다시 조회해 환경을 바꾸지 않는다.
- 타이어와 브레이크가 서로 다른 외기 온도를 사용하게 두지 않는다.
- Cool/Normal/Hot 순서를 만들기 위해 타이어 또는 브레이크 계수를 프리셋별로 바꾸지 않는다.
- 테스트 통과를 위해 타이어 상한, 진단 기준 또는 허용오차를 임의로 완화하지 않는다.
- 현재 후륜 과열 개선 결과를 환경을 차갑게 만들어 대신 통과시키지 않는다. 승인 기준은 바레인 Normal `30/40°C`다.

---

## 15. 최종 보고 형식

구현 에이전트는 다음 순서로 결과를 보고한다.

1. 변경 요약
2. 최종 데이터 계약
3. 9개 서킷 preset 표와 calibrated/provisional 상태
4. 요청 해석 우선순위와 예시
5. 타이어·브레이크 환경 전달 경로
6. 바레인 Cool/Normal/Hot 열 테스트 결과
7. 바레인 Normal 변경 전후 회귀 비교
8. 실행한 테스트와 개수
9. 실패·중단·미실행 테스트
10. 남은 위험과 C1~C5 착수 가능 여부
11. 실제 수정 파일 목록

완료를 주장할 때는 단순히 “테스트 통과”라고 쓰지 말고 주요 수치를 함께 기록한다.

---

## 16. 다른 에이전트에 전달할 실행 프롬프트

```text
/Users/kimyongjin/Desktop/f1 프로젝트에서 아래 지시서를 기준으로 서킷 열환경 기반 1차 작업을 구현해줘.

반드시 먼저 전체 지시서를 읽어:
/Users/kimyongjin/Desktop/f1/docs/CIRCUIT_THERMAL_ENVIRONMENT_PHASE_1_DIRECTIVE.md

핵심 목표:
- C1~C5 구현 전에 현재 9개 서킷에 정적 COOL/NORMAL/HOT 대기·노면 온도 프리셋을 추가
- 기존 Circuit.track_conditions는 지형 샘플이므로 절대 재사용하지 않고 Circuit.thermal_profile을 별도로 추가
- 환경값은 세션 생성 시 한 번만 해석하고 경기 중 불변으로 유지
- Race Setup UI, 예선 요청, 레이스 요청, API 응답, RaceInfo, tick, diagnostics가 같은 환경 계약을 사용
- 타이어와 브레이크가 같은 세션 대기 온도를 사용하도록 고정 30°C 브레이크 냉각 기준 제거
- 바레인 NORMAL 30/40°C와 기존 장기 열 회귀 보존

작업 전 주의:
- 현재 작업 트리는 dirty 상태다. 기존 사용자 변경을 되돌리거나 관련 없는 파일을 정리하지 마.
- 별도 요청 없이는 커밋하거나 푸시하지 마.
- 구현 전에 현재 schemas.py, data_loader.py, session.py, main.py, RaceSetup.jsx, tire_model/strategy_ops, brake_model과 관련 테스트를 읽어.
- circuit_thermal_profiles.json과 CIRCUIT_THERMAL_PRESET_SOURCES.md를 만들고, 9개 서킷 모두 명시적 프로필을 갖게 해.
- 웹에서 값을 조사할 때 FIA, Formula 1, Pirelli, 공인 기상기관 등 1차 자료를 우선하고 직접 URL을 근거 문서에 남겨.
- 실제 공식값이 부족한 수치는 provisional 시나리오라고 표시하고 실제 측정값처럼 주장하지 마.

요청 해석 규칙:
1) explicit track_conditions
2) requested thermal_preset
3) circuit default_preset
단, preset과 explicit track_conditions를 동시에 보내면 400으로 거부해.

범위 밖:
- C1~C5 물성 및 주말 매핑
- 타이어 열계수, 그립, 마모율, 열용량 보정
- 브레이크 발열·냉각 계수 보정
- 동적 날씨, 비, 일사량, 시간대 변화
- 환경에 따른 랩타임 보정

구현은 지시서의 9번 순서대로 진행하고 11번 필수 테스트와 13번 승인 기준을 모두 확인해. 특히 다음을 수치로 증명해:
- 9개 서킷 데이터 완전성
- 바레인 COOL 24/32, NORMAL 30/40, HOT 36/52°C
- 동일 입력에서 타이어와 브레이크 최종 온도 COOL < NORMAL < HOT
- 바레인 NORMAL 변경 전후 장기 회귀 허용 범위
- 정상 시나리오 타이어 hard clamp 0

최종 답변은 지시서 15번 형식으로 작성하고, 실행하지 못한 테스트나 provisional 데이터는 숨기지 말고 명시해.
```
