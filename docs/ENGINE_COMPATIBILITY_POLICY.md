# 엔진 runtime 호환 경로 정책

상태: **`engine-runtime-compat-v1` 적용**
기준일: **2026-08-12**

## 목적

FULL과 ABSTRACT의 구현 권위는 각각 `backend/engines/full/runtime/`과
`backend/engines/abstract/runtime/`에 있다. 이전 `backend/simulation/` 경로는 기존 외부 스크립트와
과거 패키지 통합을 깨뜨리지 않기 위한 호환 계층이며 새 코드의 구현 권위가 아니다.

호환 목록의 단일 코드 계약은 `backend/engines/compatibility.py`다. FULL 28개와 ABSTRACT 16개
leaf module은 legacy import와 runtime import가 동일 module 객체를 반환한다. FULL qualifying과
ABSTRACT package root는 공개 symbol을 재수출하는 facade로 유지한다.

## 현재 정책

- 저장소의 제품·도구·일반 테스트는 `engines.*.runtime`을 직접 사용한다.
- 공용 track geometry·compiler·display·data validation·vehicle dimension은 계속
  엔진 중립 `simulation.*` 계약을 사용한다.
- legacy shim은 경고를 출력하지 않는다. 실시간 앱 로그와 진단 결과를 오염시키지 않기 위함이다.
- desktop 패키징은 backend 전체를 복사하므로 runtime과 shim을 모두 포함한다.
- 경계 테스트는 manifest의 파일 존재, module identity, patch 전달과 저장소 내부 권위 사용처 0을
  검사한다.

## 삭제 조건

shim 삭제는 단순 파일 정리가 아니라 breaking change다. 다음 조건을 모두 만족한 별도 변경에서만
진행한다.

1. 저장소 내부 권위 import 0이 계속 유지된다.
2. compatibility import 없이 macOS 패키지 smoke test가 통과한다.
3. 외부 진단 스크립트와 저장된 통합 경로에 migration 안내가 제공된다.
4. 제거를 명시적 breaking change로 승인한다.

현재는 외부 사용 현황을 증명할 자료가 없으므로 shim을 유지한다. 앱 성능에는 계산 경로가 하나뿐이라
추가 물리 tick이나 메모리 복제가 발생하지 않는다.
