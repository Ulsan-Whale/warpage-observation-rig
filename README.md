# 다층 시편 열변형 관찰 장비 (Warpage Observation Rig)

## 1. 장비 개요 (Overview)
- **목적:** 반도체 패키징 리플로우 (Reflow) 가열 중 이종 재료 (Si/Cu) 의 열팽창계수 (CTE) 차이로 발생하는 열변형 (Thermal Warpage) 을 정량 관찰하는 교육용 장비 설계 및 디지털 트윈 (Digital Twin) 구현
- **기준 시편:** 30 × 30 × 0.6 mm (상부 Si 0.3 mm / 하부 Cu 0.3 mm)
- **핵심 평가 KPI:** 온도 균일도 ΔT ≤ 5℃, 지지대 영향 휨 변화율 ≤ 10%, 광학 확대불확도 ≤ 25 μm

## 2. 저장소 폴더 구조 (Repository Structure)
- `/docs` : 중간고사 (Rev.B) · 기말고사 (Rev.C/D) 발표 자료 및 기술 보고서
- `/cad-fem` : 3D CAD 모델, Fusion 열해석, SimScale 열변형 · 접촉 해석 데이터
- `/src` : PID 온도 제어 로직, 안전 인터록 (Interlock) 에러 핸들링, 섀도 모아레 시뮬레이션 GUI 코드
- `/media` : 해석 결과 그래프 및 음성 설명 시뮬레이션 동영상

## 3. 브랜치 운영 규칙 (Branch Strategy)
- `main` : 검증 완료된 최종 산출물 및 문서
- `hw-cad-fem` : 기구 설계 고도화 (Rev.C), 열해석 및 비선형 접촉 해석
- `sw-control-gui` : 파이썬 (Python) 제어 알고리즘 및 디지털 트윈 GUI 개발

## 4. 주요 트러블슈팅 로그 (Troubleshooting Log)
- **Issue #1:** 3-2-1 점 구속 vs 실제 SUS304 평행 지지대 들뜸 (Lift-off) 및 접촉 열저항 문제 분석
