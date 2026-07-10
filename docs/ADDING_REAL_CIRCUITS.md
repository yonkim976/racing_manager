# 실제 F1 서킷 데이터 추가 가이드

이 문서는 F1 Race Manager에 실제 서킷을 추가할 때 동일한 데이터 구조와 검증 수준을 유지하기 위한 작업 기준이다.

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

## 9. 향후 트랙 시각화 로드맵

현재 레이스 화면은 실제 중심선과 피트 레인 형상을 사용하지만 노면 세부 요소는 정밀 데이터 기반이 아니다.

- 트랙 노면은 중심선을 여러 굵기의 선으로 겹쳐 표현한다.
- 화면의 붉은색·흰색 연석은 모든 서킷에 동일한 진행률 구간을 적용한 장식이다. 실제 위치, 길이, 좌우 방향과 일치하지 않는다.
- 백엔드의 `track_boundaries`는 프런트엔드 렌더러에 전달하거나 그리지 않는다.
- OSM 기반 실제 서킷의 경계는 현재 실제 가변 폭 대신 기본 `12m` 폭으로 생성한다.
- `racing_line` 입력 구조와 TUM CSV 가져오기 도구는 있지만 컴파일, API, 화면 렌더링에는 연결되지 않았다.
- 런오프, 잔디, 자갈, 벽, 펜스, 마셜 포스트, FIA 라이트 패널, 섹터 라인과 스피드 트랩은 표시하지 않는다.

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
   - TUM 레이싱 라인이 있으면 중심선·경계와 정합해 사용한다.
   - 원본이 없으면 트랙 경계 안에서 최소 곡률 라인을 계산하고 차량 진행 방향과 연결한다.
   - 실제 차량 위치 계산과는 분리하고 필요할 때만 표시할 수 있는 렌더 레이어로 만든다.
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

- [ ] `track_boundaries` API 전송 및 노면 폴리곤 렌더링
- [ ] 실제 서킷별 가변 폭 데이터 연결
- [ ] 서킷별 연석 데이터 스키마와 렌더러 구현
- [ ] 기존 공통 장식 연석 제거
- [ ] 레이싱 라인 컴파일·API·표시 레이어 연결
- [ ] DRS 선·섹터·스피드 트랩 데이터 추가
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
