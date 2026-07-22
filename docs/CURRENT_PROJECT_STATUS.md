# F1 2D 레이스 시뮬레이션 현재 상태와 로드맵

상태: **현재 프로젝트 요약**
기준일: **2026-07-22**
우선 기준: 제품·물리 결정은 [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md)를 따른다.

이 문서는 지금까지 확정하고 구현한 내용을 빠르게 파악하기 위한 단일 현황 문서다. 완료된 작업의 과거 지시서와 단계별 계획서는 유지하지 않으며, 앞으로 상태가 바뀌면 이 문서와 기준 설계를 함께 갱신한다.

## 1. 제품 목표

- 실제 차량 상태를 기반으로 움직이는 **2D F1 레이스 시뮬레이션**을 만든다.
- 물리엔진이 차량의 위치·속도·헤딩·요·타이어·접촉 결과를 확정하고, 텔레메트리·그래픽·순위·이벤트는 그 상태에서 파생한다.
- AI는 추월 성공이나 사고 결과를 확률로 직접 선택하지 않는다. 공격·방어·철회, 목표 경로와 제어 입력을 선택하고 물리가 결과를 결정한다.
- 첫 완성 대상은 Bahrain International Circuit이다. 다른 트랙은 바레인에서 검증한 데이터·물리·AI 승인 절차로 확장한다.
- 현재 런타임은 모든 트랙을 평면으로 취급하는 `planar_2d`다. 고도·경사·뱅킹은 사용하지 않는다.

## 2. 확정된 차량 기준

| 항목 | 현재 기준 |
|---|---|
| 동력원 | 모든 팀 동일 750kW 단일 내연기관 기준 |
| 2026 하이브리드 | 배터리, 회생, 전기 부스트와 액티브 에어로는 후속 ruleset |
| 기준 질량 | 768kg, 연료 소모에 따른 질량 변화 적용 |
| 차체 폭 | 1.900m |
| 휠베이스 | 3.400m |
| 임시 전체 길이 | 충돌·렌더링용 5.000m |
| 물리 차원 | XY 평면, 고도 0m, 경사 0, 뱅킹 0° |

팀별 차이는 엔진 출력을 임의로 바꾸는 대신 공력, 구동계 효율, 브레이크, 기계적 그립, 트랙션과 드라이버 능력에서 만든다.

## 3. 런타임 구조

| 계층 | 주기 | 역할 |
|---|---:|---|
| 차량 물리 | 50Hz, 0.02초 고정 스텝 | 종·횡 힘, 위치, 속도, 헤딩, 요, 타이어, 접촉 |
| AI 전술 | 기본 10Hz | 공격·방어·철회, 피트와 페이스 판단 |
| 로컬 플래너 | 상황별 5/10/20/50Hz | 일반·교통·배틀·긴급 회피 경로 검증 |
| 화면 전송 | 실시간 30Hz | 최신 상태와 직전 전송 뒤의 50Hz pose 열 |
| 브라우저 렌더링 | 최대 60FPS | 120ms 재생 버퍼와 물리 pose 보간 |

지원 배속은 `1x/2x/3x`다. 배속은 한 프레임에 더 큰 물리 간격을 사용하는 방식이 아니라 같은 0.02초 물리 스텝을 더 많이 계산하는 방식이다. 같은 시뮬레이션 시간과 seed에서는 배속·외부 tick 분할과 무관하게 같은 결과를 목표로 한다.

백엔드 `RaceSession`은 시뮬레이션 credit과 30Hz 전송 deadline을 분리한다. 각 전송에는 누락되지 않은 `trajectory_samples`가 포함되고, 프런트는 `world_x_m`, `world_y_m`, `heading_rad`를 권위 pose로 사용한다.

## 4. 현재 구현된 물리

### 4.1 차량 동역학

- 750kW 출력, 구동계 효율, 질량, 구름저항, 속도별 항력과 다운포스
- 구동력·제동력·횡력에 공통으로 적용되는 결합 마찰 한계
- 전·후축 슬립각과 횡력, 조향각, 요 관성, 헤딩과 요레이트를 적분하는 평면 dynamic bicycle model
- 타이어 컴파운드, 마모, 표면·코어 온도에 따른 종·횡 그립 변화
- 연료 소모에 따른 차량 질량 변화
- 물리 입력 한계 초과로 발생하는 락업, 휠스핀, 언더스티어, 오버스티어와 run-wide
- 차체 회전 사각형 SAT와 swept collision을 이용한 접촉·고속 관통 방지
- 접촉 심각도, 정지 차량 위험물, 회피 또는 정지 선택

최근 코너에서 차체 뒤가 과도하게 드리프트하는 것처럼 보이던 문제는 실제 차체 헤딩과 그래픽 회전을 같은 pose에서 사용하도록 정리하고, rear slip/yaw 응답을 F1 차량에 맞게 안정화하는 방향으로 보정했다. 그래픽은 별도의 slip-angle 회전 필터로 물리 헤딩보다 늦게 따라가지 않는다.

### 4.2 트랙과 레이싱라인

- OSM 중심선·피트레인을 로컬 미터 좌표로 투영하고 공식 길이에 맞춰 보정
- 트랙 곡률·폭·표면 경계를 사용한 차량별 전역 궤적 최적화
- 레이싱라인은 강제 경로가 아니라 로컬 플래너의 기준 비용과 prior
- 교통이 있으면 현재 corridor를 중심으로 후보 경로를 만들고 2/3/4-wide 공간을 평가
- 경계, 연석, 아스팔트 런오프, 잔디와 자갈 표면 비용 및 트랙 리밋 판정
- Bahrain 가변 폭 14–22m와 T1 22m 적용
- Red Bull Ring TUM 기반 좌·우 폭과 2025 예선 텔레메트리 prior 적용
- Silverstone·Spa·Hungaroring은 실제 전체 경계가 없어 일부 fallback 사용

새 트랙에서도 중심선·폭·표면·차량 사양으로 최적선을 생성할 수 있다. 다만 실제 경계와 연석 정보가 부족한 결과는 “현재 모델 안의 최적선”이며, 선수 텔레메트리는 강제 경로가 아니라 속도·제동점·횡 위치 검증 자료로 사용한다.

### 4.3 피트, 출발과 경기 운영

- 서버가 `grid → lights → lights_out → racing` 상태를 단일 기준으로 관리
- 라이트가 꺼지기 전 차량을 실제 그리드 박스 pose에 배치
- 피트 진입 분기, 제한 시작선, 박스, 제한 해제선, 출구 도로와 측면 합류를 하나의 연속 경로로 계산
- 제한 시작선에 60km/h로 맞추도록 남은 거리 기반 감속
- 제한 해제 뒤 출구 도로에서 가속하고 본선 차량·ManeuverGroup 점유를 확인해 `yield → hold → merge`
- 피트와 트랙 전환 이벤트도 최종 물리 pose를 사용해 순간이동 방지
- SC/VSC, 피트 출동, 대열 수집, 언랩, 재시작과 피트 복귀 지원

현재 60km/h는 승인된 프로젝트 규칙값이다. 실제 이벤트 규정 모드에서는 서킷·이벤트 데이터로 분리해야 한다.

## 5. AI와 레이스크래프트

현재 AI는 다음 계층으로 나뉜다.

1. 전역 최적선과 속도 프로파일
2. 점유·충돌·경계를 보는 온라인 로컬 궤적 플래너
3. 공격·방어·철회와 위험을 판단하는 전술 계층
4. 경로를 조향·스로틀·브레이크로 추종하는 저수준 제어기
5. 모든 결과를 확정하는 공통 물리엔진

구현된 항목:

- 현재 직선 길이, 다음 제동구간, 접근속도, DRS, 웨이크와 필요 추월거리 기반 공격 판단
- `attack`, `hold`, `abort`와 설명 가능한 `reason_code`
- 두 차량 pair가 아니라 최대 4대의 `ManeuverGroup`으로 corridor와 코너 우선권 관리
- front-axle 위치와 inside corridor를 사용한 코너 진입 우선순위
- 실제 차체 clearance가 생겨야 추월 완료
- 폭이 부족하거나 비용이 높으면 접촉 없이 철회
- 120m 내 여러 차량 중 가장 강하게 정렬된 후류 원천 선택
- 직선에서는 tow, 곡률이 충분한 코너에서는 dirty-air의 다운포스·제동·횡그립 손실
- 타이어와 연료 자원을 고려한 AI 페이스·피트 전략

바레인 v1 판단 벤치마크는 [`AI_RACECRAFT_BENCHMARK.md`](AI_RACECRAFT_BENCHMARK.md)에 기록한다. 아직 실제 F1 리플레이와 추월 시도·성공·철회·접촉 빈도 분포가 일치한다고 보지는 않는다.

## 6. 타이밍과 텔레메트리

- 트랙을 공식 3개 대섹터와 각 6개 미니섹터, 총 18개 타이밍 루프로 구성
- 50Hz 프레임 안의 타이밍 선 통과 시각을 보간
- GAP/INT는 최신 공통 타이밍 루프의 실제 통과 시각 차이를 사용
- 공통 기록이 아직 없을 때만 추정값을 사용하고 `timing_gap_valid=false`로 구분
- 랩 기록에 3개 sector time과 18개 mini-sector time 보존
- 차량 pose, 속도·가속도, 스로틀·브레이크, 요·슬립, 타이어, 웨이크, 피트·배틀·접촉 상태를 전송

## 7. 현재 그래픽 구조

현재 트랙 화면은 React 안의 PixiJS 8 WebGL 렌더러다.

- Pixi ticker 최대 60FPS, device pixel ratio 최대 2
- 50Hz pose 열을 약 120ms 늦춰 재생해 패킷 사이를 cubic Hermite 곡선으로 보간
- 최대 120ms·45m의 제한적 dead reckoning은 패킷이 늦을 때만 사용
- 메인 확대 배율 `700/1000/1500%`, 기본 `1000%`
- 100% 전체 트랙 미니맵은 별도 overview로 유지
- 가변 폭 아스팔트 폴리곤, 실제 표면 구역, 피트 도로, 그리드, 대·미니섹터 표시
- 차량 pose와 카메라 추적을 같은 60FPS 재생 상태에서 계산
- 정적 트랙과 차량 pose 갱신을 분리하고 TrackCanvas를 lazy chunk로 로드

현재 PixiJS도 이미 WebGL을 사용한다. 따라서 Three.js 전환은 GPU 가속을 새로 켜는 작업이 아니라 2D 장면을 2.5D/3D 장면으로 교체하는 작업이다.

### Three.js 렌더러 방향

현실적인 차량 형태, 도로 재질, 높이가 있는 연석·장벽, 조명과 접지 그림자를 강화할 때는 다음 구조를 사용한다.

- 물리·AI·WebSocket과 pose 재생 버퍼는 유지한다.
- Three.js `OrthographicCamera`를 사용하는 바레인 전용 `ThreeTrackCanvas`를 기능 플래그로 먼저 만든다.
- 물리 `world_x_m/world_y_m`는 Three.js X/Z 평면에 매핑하고 Y는 시각적 높이에만 사용한다.
- 트랙·연석·런오프는 시작 시 한 번 생성하는 정적 Mesh로 만든다.
- 20대 차량은 공유 geometry와 instance color 또는 소수 재질을 사용한다.
- React 대시보드와 SVG 미니맵은 유지한다.
- 전환 기간 외에는 PixiJS와 Three.js가 같은 트랙 장면을 장기적으로 이중 렌더링하지 않는다.
- 동적 고해상도 그림자·모션 블러·과도한 후처리보다 baked/fake shadow와 정적 배치를 우선한다.

Three.js 프로토타입도 기존 30Hz 전송과 50Hz pose 열을 사용하며 패킷 주기를 올리지 않는다. 1/2/3배속·20대·700/1000/1500%에서 60FPS 프레임 예산과 메모리를 기존 Pixi 화면과 비교한 뒤 기본 렌더러 전환 여부를 결정한다.

## 8. 바레인 검증 상태

현재 저장된 주요 검증 결과:

- 1x/2x/3x 각 200초, 총 10분 런타임 soak 통과
- 3x 벽시계 600초, 시뮬레이션 30분·89,999 물리 프레임 플래너 soak 통과
- 30분 결과: 실효 배속 중앙값 3.000x, backlog p95 18.3ms, broadcast jitter p95 13.7ms
- trajectory 최대 50Hz 이동 1.77m, frame gap 1
- 바레인 2/3/4-wide 고정 스텝 분할 결정성 golden 통과
- 피트 진입부터 측면 합류까지 단일 50Hz 종단 golden 통과
- 직선 tow와 T1 dirty-air 수치 golden 통과
- 최근 전체 회귀 기록은 Backend 380개 테스트, frontend lint/build 통과

성능과 보정 상세 자료는 `backend/data/calibration/`에 저장한다. 이 문서 정리 작업에서는 전체 테스트를 다시 실행하지 않았으며 위 수치는 직전 구현 검증 기록이다.

## 9. 남아 있는 핵심 한계

- 실제 F1 리플레이 분포에 맞춘 공격 시도·성공·철회·접촉·forced-wide 빈도 보정이 필요하다.
- Bahrain T1/T2 코너 최저속도 오차와 세션별 노면·기온·바람 차이가 남아 있다.
- 타이어 압력, 하중 이동, 브레이크 열과 바퀴별 상세 slip ratio는 아직 없다.
- Silverstone·Spa·Hungaroring은 실제 좌우 경계·연석·표면 데이터가 부족하다.
- 팀별 직선·저속·고속·제동 특성과 컴파운드별 스틴트 길이 벤치마크가 부족하다.
- 실제 브라우저에서 20대 장시간 프레임타임·메모리 계측이 더 필요하다.
- Three.js는 도입 가능성만 검토했으며 아직 의존성이나 구현이 추가되지 않았다.
- 2026 하이브리드 파워유닛, 에너지 관리, 액티브 에어로, 날씨와 젖은 노면은 범위 밖이다.

## 10. 다음 권장 순서

1. 현재 Pixi 렌더러와 pose/카메라/트랙 geometry adapter를 분리한다.
2. Bahrain `ThreeTrackCanvas` 기능 플래그 프로토타입을 만들고 Pixi 버전과 60FPS·메모리·화질을 비교한다.
3. 실제 Bahrain 리플레이로 코너별 공격·철회·성공·접촉 분포를 라벨링하고 AI 벤치마크 v2를 만든다.
4. Bahrain T1/T2 속도·제동·스로틀 오차를 재검증한다.
5. 타이어 압력, 하중 이동과 브레이크 열을 독립 회귀 테스트와 함께 추가한다.
6. Silverstone·Spa·Hungaroring의 실제 경계·연석·표면을 순차 적용한다.
7. 새 트랙 가져오기, 전역 라인 생성, 단독 안전 랩과 실제 데이터 비교를 하나의 승인 명령으로 묶는다.
8. 2026 파워유닛·에너지·액티브 에어로는 별도 ruleset과 문서로 시작한다.

## 11. 주요 코드 위치

### Backend

- `backend/session.py`: 고정 스텝 세션 루프, 30Hz 전송, pose 수집
- `backend/simulation/race_engine.py`: 레이스 상태, AI·피트·타이밍·규칙 통합
- `backend/simulation/vehicle_physics.py`: 종방향 힘과 저수준 차량 제어
- `backend/simulation/vehicle_dynamics.py`: dynamic bicycle model
- `backend/simulation/global_trajectory_optimizer.py`: 전역 궤적 최적화
- `backend/simulation/local_trajectory_planner.py`: 교통·추월·회피 후보 경로
- `backend/simulation/planner_scheduler.py`: 상황별 플래너 주기와 캐시
- `backend/simulation/collision.py`: OBB/SAT와 swept collision
- `backend/simulation/wake_model.py`: tow와 dirty air
- `backend/simulation/track_surface.py`: 경계·연석·런오프 표면
- `backend/simulation/track_data_validation.py`: 트랙 데이터 계보와 승인 검사

### Frontend

- `frontend/src/components/TrackView/TrackCanvas.jsx`: PixiJS WebGL 트랙과 차량 렌더링
- `frontend/src/hooks/useRaceWebSocket.js`: 30Hz pose ref와 10Hz React UI 상태 분리
- `frontend/src/components/Dashboard/TimingBoard.jsx`: GAP/INT와 섹터 타이밍
- `frontend/src/components/Dashboard/StrategyPanel.jsx`: 타이어·페이스·피트 제어

## 12. 실행과 검증

Backend 전체 테스트:

```bash
cd backend
.venv/bin/python -m unittest discover -s tests
```

바레인 레이스크래프트 벤치마크:

```bash
cd backend
.venv/bin/python tools/run_bahrain_racecraft_benchmark.py --seeds 10 --seconds 2
```

바레인 런타임 soak:

```bash
cd backend
.venv/bin/python tools/run_bahrain_runtime_soak.py --help
```

Frontend 검증:

```bash
cd frontend
npm run lint
npm run build
```

새 실제 서킷 추가 절차는 [`ADDING_REAL_CIRCUITS.md`](ADDING_REAL_CIRCUITS.md)를 따른다.
