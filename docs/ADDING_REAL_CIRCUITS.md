# 실제 F1 서킷 데이터 추가 가이드

이 문서는 F1 Race Manager에 실제 서킷을 추가할 때 동일한 데이터 구조와 검증 수준을 유지하기 위한 작업 기준이다.

> 문서 역할: 실제 서킷 가져오기 작업 절차. 제품 물리, 데이터 권위 관계와 레이싱라인 채택 정책은 [`SIMULATION_FOUNDATION.md`](SIMULATION_FOUNDATION.md)를 우선한다.

## 1. 데이터 구조

실제 서킷은 게임 설정과 원본 좌표를 분리한다.

- `backend/data/circuits.json`: 랩 수, 공식 길이, 피트 손실, DRS, 코너, 주행 구간 등 게임 메타데이터
- `backend/data/circuit_sources/*_osm_geo.json`: OSM 기반 WGS84 중심선과 피트 레인 원본
- `backend/data_loader.py`: `geo_file`을 읽어 `Circuit.geo`에 결합
- `backend/simulation/geo_projection.py`: WGS84를 로컬 미터로 투영하고 공식 길이에 맞춰 보정
- `backend/simulation/track_compiler.py`: 렌더 좌표, 피트 진입·출구 앵커, 트랙 경계와 랜드마크 인덱스 생성

소스 우선순위는 `metric -> geo -> editor/layout`이다. 실제 F1 서킷은 특별한 이유가 없으면 현재 운영 방식인 `geo_file`을 사용한다.

## 2. 권장 데이터 소스

1. FIA 최신 이벤트 서킷 맵
   - 공식 중심선 길이, 코너 번호, DRS 검출·활성 지점 확인
2. Formula 1 공식 서킷 페이지
   - 랩 수, 레이스 거리, 기준 랩타임 확인
3. OpenStreetMap `highway=raceway`
   - 실제 중심선과 피트 레인 WGS84 원본
   - 결과 파일에 ODbL과 `OpenStreetMap contributors` 표기 유지
4. TUMFTM racetrack-database
   - OSM 중심선의 형상 교차 검증과 트랙 폭 참고
   - LGPL-3.0 데이터를 프로젝트에 직접 포함할 때는 라이선스 조건을 별도로 검토

게임 원본은 한 출처를 기준으로 만들고 다른 출처는 검증에 사용한다. 서로 다른 중심선을 부분적으로 이어 붙이지 않는다.

Red Bull Ring의 구현 예시는 중심선·피트에 OSM/ODbL, 좌·우 폭에 [TUMFTM Spielberg.csv](https://github.com/TUMFTM/racetrack-database/blob/master/tracks/Spielberg.csv)/LGPL-3.0, 속도·스로틀·브레이크에 [TracingInsights 2025 Austrian GP Qualifying](https://github.com/TracingInsights-Archive/2025/tree/main/Austrian%20Grand%20Prix/Qualifying)을 사용한다. 텔레메트리는 상위 5개 독립 드라이버 랩의 median이다.

`backend/tools/fetch_red_bull_ring_references.py`는 고정된 공개 URL을 읽어 compact JSON을 stdout으로 만들며 프로젝트 파일을 직접 덮어쓰지 않는다. 생성 결과는 출처·라이선스·정렬 오차와 물리 비교 리포트를 검토한 뒤 적용한다.

## 3. OSM 원본 생성

먼저 서킷 주변의 raceway way를 조회해 다음 값을 확인한다.

- 그랑프리 레이아웃을 구성하는 폐곡선
- 주행 방향
- 출발선과 가장 가까운 위도·경도
- F1 피트 레인 way ID
- 카트·모터사이클·쇼트 레이아웃 제외 여부

`backend` 디렉터리에서 아래 도구를 사용한다.

```bash
.venv/bin/python tools/import_osm_circuit.py \
  --bbox MIN_LAT,MIN_LON,MAX_LAT,MAX_LON \
  --official-length-m OFFICIAL_LENGTH \
  --start-lonlat START_LAT,START_LON \
  --pit-way-id PIT_WAY_ID \
  --source-id osm:CIRCUIT_SLUG-gp \
  --pretty \
  --output data/circuit_sources/CIRCUIT_SLUG_osm_geo.json
```

OSM way 방향이 끊어지는 경우에만 `--allow-reverse-ways`를 추가한다. 완성된 파일은 다음 필드를 포함해야 한다.

```json
{
  "source": "osm",
  "sourceId": "osm:CIRCUIT_SLUG-gp",
  "license": "ODbL",
  "attribution": "OpenStreetMap contributors",
  "centerlineLonLat": [],
  "pitLaneLonLat": [],
  "sourceLengthM": 0,
  "scaleToOfficialLength": true,
  "sampleSpacing": 16,
  "pitSampleSpacing": 14,
  "officialLengthM": 0
}
```

`centerlineLonLat[0]`은 출발선이어야 하며 마지막 점은 첫 점과 같아야 한다. 피트 레인은 실제 주행 방향으로 진입점부터 출구점까지 정렬한다.

## 4. 게임 메타데이터 작성

`backend/data/circuits.json`에 다음 항목을 추가한다.

- 고유 `id`, 공식 명칭과 국가
- `base_lap_time`, `total_laps`, `pit_loss_time`, `track_length_m`
- `geo_file`
- 3개 섹터
- `pit_lane.wall_offset`, `pit_lane.box_offset`
- FIA 기준 DRS 활성 구간
- 출발선, 피트, 모든 코너 랜드마크
- 0.0부터 1.0까지 빈틈없이 이어지는 주행 세그먼트

세그먼트 규칙:

- 각 세그먼트 길이는 진행률 `0.03` 이상
- 첫 세그먼트는 `0.0`에서 시작하고 마지막 세그먼트는 `1.0`에서 종료
- DRS 구간은 반드시 `straight` 세그먼트 안에 위치
- 급제동, 기술 구간, 트랙션, 고속 스위핑을 실제 특성에 맞게 구분
- `speed_factor`는 기존 서킷 범위와 비교해 과도하게 설정하지 않음

랜드마크 진행률은 OSM 중심선의 누적 거리와 FIA 코너 지점을 비교해 정한다. 화면상의 수동 좌표를 사용하지 않는다.

## 5. 필수 수치 검증

```bash
cd backend
.venv/bin/python -m unittest discover -s tests
```

승인 기준:

- JSON 파싱 및 Pydantic 검증 성공
- 컴파일 중심선 길이가 공식 길이와 `0.5m` 이내
- 원본 중심선 길이가 공식 길이와 `1%` 이내
- 컴파일된 중심선 80점 이상, 피트 레인 8점 이상
- 중심선 최대 점 간격 24 렌더 단위 미만
- 피트 시작·끝을 메인 트랙의 최근접 선분 위에 투영했을 때 오차 0.01 렌더 단위 미만
- 피트 출구 진행 방향과 메인 트랙 진행 방향이 동일
- DRS 시작과 끝이 `straight` 세그먼트에 포함
- 모든 랜드마크 인덱스가 컴파일 중심선 범위 안에 존재

가능하면 TUM 중심선과 폐곡선 정합을 수행한다. 회전·이동·시작 인덱스를 정렬한 뒤 RMS, 중앙값, 95백분위 오차를 기록한다.

## 6. 화면 검증

프런트엔드에서 실제 레이스를 시작해 다음을 확인한다.

- 기본 `100%`에서 서킷과 라벨이 잘리지 않고 캔버스를 충분히 사용
- 세로로 긴 GPS 형상은 표시 전용으로 90도 회전되고 방위각 HUD가 일치
- 데이터 좌표와 차량 진행 방향은 화면 회전에 영향받지 않음
- 출발 위치와 첫 코너 진행 방향이 실제 서킷과 일치
- 피트콜 후 차량이 피트 입구, 박스, 출구를 순서대로 통과
- `track <-> pit` 전환에서 순간이동이나 역주행이 없음
- DRS 표시가 실제 직선 구간에 위치
- 브라우저 콘솔 오류와 페이지 스크롤 오버플로가 없음

화면 방향을 서킷별로 바꿔야 할 때는 GPS 원본을 수정하지 않는다. `frontend/src/App.jsx`의 표시 회전만 오버라이드하고 방위각이 같은 각도로 갱신되는지 확인한다.

## 7. 스파-프랑코르샹 적용 기록

- 공식 길이: `7,004m`
- 레이스 랩 수: `44`
- OSM 출발점: `50.4440643, 5.9651626`
- OSM F1 피트 레인 way: `323851541`
- 컴파일 피트 진입/출구 진행률: `0.946 / 0.063`
- OSM 컴파일 길이: `7,004.0m`
- TUM 중심선 길이: `7,000.1m`
- OSM 대 TUM 형상 정합: RMS `6.72m`, 중앙값 `5.89m`, P95 `11.49m`, 최대 `13.82m`
- FIA 2025 기준 DRS: Kemmel Straight, Main Straight
- 표시 회전: `270도` 오버라이드. La Source는 좌측 하단, 북쪽은 화면 왼쪽

참고 링크:

- FIA 2025 circuit map: https://www.fia.com/system/files/decision-document/2025_spa_francorchamps_event_-_circuit_map_-_spa_2025.pdf
- Formula 1 circuit page: https://www.formula1.com/en/information/belgium-circuit-de-spa-francorchamps.3LltuYaAXVRU8iezEsjzGw
- Spa-Francorchamps official circuit page: https://www.spa-francorchamps.be/en/the-circuit
- TUMFTM racetrack-database: https://github.com/TUMFTM/racetrack-database

## 8. 헝가로링 적용 기록

- 공식 길이: `4,381m`
- 레이스 랩 수: `70`
- OSM 출발점: `47.5787136, 19.2487084`
- OSM 그랑프리 레이아웃 way: `1333262244`, `231328650`
- OSM F1 피트 레인 way: `231417580` (`Bokszutca`)
- 컴파일 피트 진입/출구 진행률: `0.882 / 0.138`
- OSM 원본 중심선 길이: `4,372.1m`
- OSM 컴파일 길이: `4,381.0m`
- TUM 중심선 길이: `4,376.9m`
- OSM 대 TUM 형상 정합: RMS `1.15m`, 중앙값 `0.94m`, P95 `2.17m`, 최대 `2.76m`
- FIA 2025 기준 DRS: T14 뒤 `40m`부터 메인 스트레이트, T1 뒤 `6m`부터 T1-T2 스트레이트
- 표시 회전: 세로형 GPS 형상에 자동 `90도` 적용, 방위각 HUD `N 090도`
- 화면 검증: `1280x720`, 트랙 캔버스 `586x405`, `100%` 줌, 페이지 오버플로 없음
- 피트 검증: 실제 레이스 피트콜 후 피트 횟수 증가와 메인 트랙 복귀 확인

참고 링크:

- FIA 2025 circuit map: https://www.fia.com/system/files/decision-document/2025_budapest_event_-_circuit_map_-_budapest_2025.pdf
- Formula 1 circuit page: https://www.formula1.com/en/racing/2025/hungary
- OpenStreetMap circuit relation: https://www.openstreetmap.org/relation/284557
- TUMFTM racetrack-database: https://github.com/TUMFTM/racetrack-database/blob/master/tracks/Budapest.csv

## 몬차(Monza) 적용 기록

- 공식 길이: `5,793m`
- 레이스 랩 수: `53`
- OSM 출발점: `45.6189632, 9.2811729`
  - F1 공식 "폴에서 T1 제동점까지 472m" + 제동거리 약 140m을 Variante del Rettifilo 진입점에서 역산한 위치와, Tribuna Centrale 정면을 트랙에 투영한 위치가 일치
- OSM 그랑프리 레이아웃: 20개 way 폐곡선 (`19842206` Rettifilo di partenza 등, 원본 5,795.8m)
- OSM F1 피트 레인 way: `38168747` (`Pit Lane`, 741.4m)
- 컴파일 피트 진입/출구 진행률: `0.903 / 0.032` (피트 진입은 실제처럼 Parabolica 안쪽을 가로지름)
- OSM 원본 중심선 길이(회전 후): `5,798.0m`
- OSM 컴파일 길이: `5,793.0m`
- FIA 2025 기준 DRS: Serraglio Straight(T7 뒤 170m 활성), Main Straight(T11 뒤 20m 활성)
- 코너 진행률은 OSM 코너 way(Variante del Rettifilo, Curva Biassono, Variante della Roggia, Lesmo 1/2, Vialone·Variante Ascari, Curva Alboreto)의 누적 거리에서 도출
- 검증: `validate_track_data --circuit-id 8` 통과, 전용 단위 테스트 3개와 실서킷 공통 피트 왕복·SC 배치 테스트 통과
- 주의: 폭 프로파일 미적용(기본 12m fallback). Tier B 대기

참고 링크:

- FIA 2025 circuit map: https://www.fia.com/system/files/decision-document/2025_monza_event_-_circuit_map_-_monza_2025_0.pdf
- Formula 1 circuit page: https://www.formula1.com/en/racing/2025/italy
- Monza 공식 트랙 안내(직선 1,194.4m): https://www.monzanet.it/en/circuit/

## 잔드보르트(Zandvoort) 적용 기록

- 공식 길이: `4,259m`
- 레이스 랩 수: `72`
- OSM 출발점: `52.3902596, 4.5417272`
  - RaceFans/FIA 이벤트 데이터의 "그리드에서 T1까지 164m"를 Tarzanbocht 진입점에서 역산한 **추정값**
- OSM 그랑프리 레이아웃: 24개 way 폐곡선 (원본 4,256.5m)
- OSM F1 피트 레인 way: `38144527` (`Pitstraat`, 757.6m)
- 컴파일 피트 진입/출구 진행률: `0.907 / 0.093`
- OSM 원본 중심선 길이(회전 후): `4,261.9m`
- OSM 컴파일 길이: `4,259.0m`
- FIA 2025 기준 DRS: Back Straight(T10 뒤 50m 활성), Main Straight
  - FIA의 두 번째 활성점은 T13 뒤 40m로 뱅크드 T14 **코너 안**에 있으나, 게임 규칙(DRS는 straight 세그먼트 안)에 따라 T14 출구 뒤(0.888)로 이동. 2021년 규격과 동일한 위치이며 의도된 편차로 기록
- 코너: OSM 코너 way 이름(Tarzan, Gerlach, Hugenholtz, Hunzerug, Slotemaker, Scheivlak, Masters, CM.com bocht, Hans Ernst, Arie Luyendyk) 기준. T8/T9는 무명 way 구간에서 기하로 추정
- 검증: `validate_track_data --circuit-id 9` 통과, 전용 단위 테스트 3개와 실서킷 공통 피트 왕복·SC 배치 테스트 통과
- 주의: 폭 프로파일 미적용(기본 12m fallback). 고도·뱅킹(T3/T14 18도)은 `planar_2d` 정책상 물리에 미반영

참고 링크:

- FIA 2025 Race Director's Event Notes: https://www.fia.com/system/files/decision-document/2025_dutch_grand_prix_-_race_directors_event_notes_.pdf
- Formula 1 circuit page: https://www.formula1.com/en/racing/2025/netherlands
- RaceFans 트랙 데이터: https://www.racefans.net/f1-information/going-to-a-race/zandvoort/

## 바르셀로나-카탈루냐(Barcelona-Catalunya) 적용 기록

- 공식 길이: `4,657m`
- 레이스 랩 수: `66`
- OSM 출발점: `41.5700636, 2.2612477`
  - F1 이벤트 데이터의 "폴에서 첫 제동점까지 565m" + 제동거리 약 125m을 T1(Elf) 진입점에서 역산한 **추정값**
- OSM 그랑프리 레이아웃: 모토GP 폐곡선 way `831804327`(원본 4,680.1m, 옛 T14–15 치케인 포함)를 노드 `1300807861`/`385973423`에서 절단하고, AZ 바이패스 way `893732520`+`831804325`+`990483278`(186.5m)로 재스티치한 2025 F1 레이아웃. 원본 4,664.0m
- OSM F1 피트 레인: `33742214`(Entrada) + `178416729`(Pit Lane) + `178416733`(Sortida) 체인, 1,036.9m
- 컴파일 피트 진입/출구 진행률: `0.880 / 0.107` (피트 진입은 T13–T14 사이에서 본선을 떠나 최종 코너 안쪽을 가로지름)
- OSM 원본 중심선 길이: `4,664.0m` (공식 대비 +7.0m, 1% 이내)
- OSM 컴파일 길이: `4,657.0m`
- FIA 기준 DRS: Back Straight(T9 뒤 40m 활성), Main Straight(T14 뒤 활성, S/F 랩어라운드)
- 코너: OSM 헤딩 변화로 14개 코너 추출. 2025 레이아웃에는 치케인(구 T14–15)이 없음
- 임포터 확장: `import_osm_circuit.py`에 `--split-way`, `--pit-way-ids` 추가(치케인 바이패스·분할 피트 체인용)
- 검증: `validate_track_data` 통과, Tier A 감사 통과, 전용 단위 테스트 3개와 `tests.test_engine` 283개 전부 통과
- 주의: 폭 프로파일 미적용(기본 12m fallback). Tier B 대기

참고 링크:

- Formula 1 circuit page: https://www.formula1.com/en/racing/2025/spain
- RaceFans 트랙 데이터: https://www.racefans.net/f1-information/going-to-a-race/circuit-de-catalunya-barcelona-circuit-information/

## 스즈카(Suzuka) 적용 기록

- 공식 길이: `5,807m`
- 레이스 랩 수: `53`
- OSM 출발점: `34.8432400, 136.5403949`
  - F1 이벤트 데이터의 "폴에서 첫 제동점까지 277m" + 제동거리 약 90m을 1コーナー(way `183391643`) 진입점에서 역산한 **추정값**
- OSM 그랑프리 레이아웃: 40개 way 폐곡선 (메인/西ストレート, S字, 역뱅크, 데그너, 헤어핀, 스푼, 130R, 히타치 Astemo 치케인 등, 원본 5,811.4m)
- OSM F1 피트 레인 way: `120917578` (`Pit Lane`, 888.3m) — 치케인 출구~1코너를 본선과 평행
- 컴파일 피트 진입/출구 진행률: `0.909 / 0.063`
- OSM 원본 중심선 길이: `5,811.4m` (공식 대비 +4.4m)
- OSM 컴파일 길이: `5,807.0m`
- FIA 2025 기준 DRS: Main Straight 단일 존 (T18 뒤~T1 제동, 랩어라운드)
- 코너: OSM 이름 way(1コーナー, 2코너, S字, 역뱅크, 데그너1/2, NISSIN 헤어핀, 스푼, 130R, 치케인) + 헤딩 변화로 18개 배치
- 피겨8: `allows_self_intersection: true` — 브리지 교차 1곳을 의도된 자기교차로 허용. 검증기·단위 테스트는 교차 개수=1을 요구
- 검증: `validate_track_data` 통과, Tier A 감사 통과, 전용 단위 테스트 3개
- 주의: 폭 프로파일 미적용(기본 12m fallback). 고도·뱅킹은 `planar_2d` 정책상 미반영

참고 링크:

- Formula 1 circuit page: https://www.formula1.com/en/racing/2025/japan
- FIA 2025 media kit (5.807km / 53 laps): https://www.fia.com/sites/default/files/2025mediakit0402a.pdf

## 9. 현재 트랙 데이터 및 시각화 상태

현재 레이스 화면은 실제 중심선과 피트 레인 형상을 사용하지만 노면 세부 요소는 정밀 데이터 기반이 아니다.

- 기본 overview는 중심선을 여러 굵기의 선으로 표현하고, 700% 이상 물리 배율에서는 좌우 폭 프로파일로 닫힌 노면 폴리곤을 생성한다.
- 붉은색·흰색 연석은 공통 장식 구간을 제거하고 `surface_zones`가 제공하는 서킷별 위치·방향·폭을 사용한다. 다만 공개 정밀 측량이 없는 구간은 추정 데이터다.
- 백엔드는 `track_width_profile`, 표면 구역, 레이싱라인과 차량별 주행선 좌표를 레이스 API로 전달하고 프런트엔드는 이를 표시할 수 있다.
- Bahrain은 공식 `14–22m` 범위와 T1 `22m` 제어점으로 가변 폭을 생성한다. Red Bull Ring은 TUMFTM Spielberg 좌·우 폭 샘플을 적용한다. Silverstone·Spa·Hungaroring은 실제 좌우 폭이 없어 기본 `12m` fallback이며, 대칭 재구성을 실측 edge로 표기하면 안 된다.
- 물리 기반 레이싱라인은 차량·타이어별로 최적화되어 레이스 물리와 화면에 연결된다. 다만 실제 폭, 뱅크, 연석과 노면 데이터가 없으므로 실제 최적선으로 확정하지 않는다.
- TracingInsights 예선 위치 데이터에서 만든 횡 오프셋은 물리 최적화의 보정 기준으로 사용한다. 차량을 텔레메트리 경로에 고정하지 않는다.
- 데이터가 있는 아스팔트 런오프·잔디·자갈은 표시하고 물리 표면 비용에도 사용한다. 벽, 펜스, 마셜 포스트, FIA 라이트 패널과 스피드 트랩은 아직 없다. 대섹터와 프로젝트 미니섹터 선은 표시한다.

권장 구현 순서:

1. 실제 트랙 폭과 좌우 경계
   - `Circuit.track_boundaries`를 레이스 API에 포함한다.
   - 중심선 굵기 대신 좌우 경계로 닫힌 노면 폴리곤을 생성한다.
   - TUM의 좌우 폭 샘플을 우선 사용하고, 데이터가 없는 OSM 서킷은 구간별 폭 프로파일을 별도 메타데이터로 관리한다.
   - 경계 자기 교차, 폭 급변, 중심선 이탈을 자동 테스트한다.
2. 실제 연석
   - 연석을 `start`, `end`, `side`, `width`, `pattern` 데이터로 정의한다.
   - FIA 맵, 온보드 영상, 항공 사진을 비교해 코너별 좌우 위치를 기록한다.
   - 현재 `drawKerbs()`의 공통 고정 구간을 제거하고 서킷별 데이터만 렌더링한다.
3. 레이싱 라인
   - 실제 좌우 폭과 표면 데이터를 물리 최적화 입력으로 연결한다.
   - 최소 곡률 라인은 초기값으로 사용하고 차량·타이어별 최소 랩타임 비용으로 최적화한다.
   - TUM 또는 실제 선수 랩은 정답 경로가 아니라 형상·속도·제동점 검증과 보정 prior로 사용한다.
   - 최적화 경로와 실제 차량 주행 경로의 차이를 텔레메트리로 기록한다.
4. FIA 및 트랙 시설물
   - DRS 감지선·활성선, 섹터 라인, 스피드 트랩을 진행률 데이터로 추가한다.
   - 마셜 포스트, 라이트 패널, 피트 제한선과 트랙 경계 표식을 별도 랜드마크 타입으로 추가한다.
   - 런오프, 잔디, 자갈, 벽과 펜스는 실제 좌표를 확보할 수 있는 서킷부터 선택적으로 적용한다.
5. 화면 및 회귀 검증
   - `100%` 줌과 확대 화면에서 경계·연석·레이싱 라인이 트랙 밖으로 벗어나지 않아야 한다.
   - 표시 회전 후에도 좌우 연석과 시설물 방향이 뒤집히지 않아야 한다.
   - 데스크톱·모바일 스크린샷 비교와 캔버스 픽셀 검사를 추가한다.
   - 데이터가 없는 레이어는 기존 중심선 화면으로 안전하게 대체한다.

작업 체크리스트:

- [x] 트랙 폭 프로파일·레이싱라인·표면 구역 API 전송
- [x] 폭 프로파일 기반 노면 폴리곤 렌더링
- [ ] 실제 서킷별 가변 폭 데이터 연결 — Bahrain·Red Bull Ring 완료, 3개 트랙 대기
- [x] 서킷별 연석·런오프 표면 구역 스키마와 렌더러 구현
- [x] 기존 공통 장식 연석 제거
- [x] 물리 레이싱라인 컴파일·API·표시 레이어 연결
- [ ] 실제 랩 비교 및 트랙별 품질 리포트
- [ ] DRS 선·섹터·스피드 트랩 데이터 추가 — DRS와 3대섹터·18 미니섹터 완료, 스피드 트랩 대기
- [ ] 마셜 포스트·라이트 패널·트랙 경계 표식 추가
- [ ] 런오프·잔디·자갈·벽·펜스 선택적 지원
- [ ] 경계 및 레이어 자동 테스트와 화면 회귀 테스트

## 10. 서킷 추가 완료 체크리스트

- [ ] 공식 제원과 최신 FIA 맵 확인
- [ ] OSM 중심선·피트 레인 방향 확인
- [ ] `circuit_sources` 원본 및 라이선스 표기 추가
- [ ] `circuits.json` 메타데이터 추가
- [ ] 전용 길이·피트·DRS 테스트 추가
- [ ] 전체 백엔드 테스트 통과
- [ ] API 목록과 레이스 생성 확인
- [ ] 데스크톱 화면과 방위각 확인
- [ ] 실제 피트콜 진입·이탈 확인
