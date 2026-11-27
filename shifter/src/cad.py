# LSK H/SEQ Shifter — author: Sunggwan
"""v0.7 최종 좌표 (모든 산출물·검사·렌더는 이 모듈만 쓴다).

왜 이렇게 나눴나 (11-27 Fusion 검토):
  v0.6 책상 클램프에서 모드 노브 면(+X)을 운전석 쪽으로 정했는데, 기구·게이트 각인·펌웨어는 v0.1부터
  "+X = 차 앞(운전자에서 먼 쪽), +Y = 운전자 왼쪽" 기준이었다 → 운전석에서 보면 H 패턴이 180° 돌아가 있었다.
  사용자 결정: 기구를 좌우(Y) 반전해서 +X 운전석에서 표준 7+R (1-2 칸 왼쪽, 7-R 칸 오른쪽, 홀수 단 먼 쪽).

구조:
  cad_model.py = 반전 전 "모델 좌표" (v0.6까지의 숫자 그대로, 글자 각인만 미리 뒤집어 둠)
  cad.py       = 모델 형상을 XZ 평면 기준으로 뒤집고(Y → −Y), −Y/+Y 이름을 바꿔 붙인 최종 형상

최종 좌표: 원점 = 짐벌 피벗. +X = 운전석 쪽(모드 노브 면), −X = 먼 쪽(USB-C), +Z = 위.
         +Y = 운전자 오른쪽(7-R 칸·캠·작은 창 쪽), −Y = 운전자 왼쪽(1-2 칸·스프링 카트리지 S2 접근 구멍 쪽).
         PHI(최종) = −PHI_M: 1-2 −15.26° · 3-4 0° · 5-6 +15.26° · 7-R +28.61°. θ +는 레버 윗끝이 +X(몸 쪽).
주의: cad_model.py의 Y 관련 상수(YC, SHAFT_Y, GATE_Y, Y_NEG_OUT …)는 반전 전 값 → 최종 좌표에서는 부호를 바꿔 읽는다.
"""
from cad_model import *                       # noqa: F401,F403 — 상수·도우미 (X·Z 값은 그대로 유효)
import cad_model as M                         # (별표 가져오기 뒤에 — cad_model 안에도 M이라는 이름이 있음)
from build123d import Plane
from shapely import affinity

MIRROR_Y = True
_SWAP = {"tower_negY": "tower_posY", "tower_posY": "tower_negY",
         "panel_negY": "panel_posY", "panel_posY": "panel_negY",
         "plunger_S1_L": "plunger_S1_R", "plunger_S1_R": "plunger_S1_L"}
_XY_PLATES = {"base", "gate", "clamp_plate"}   # 2D 윤곽이 XY 평면인 판 → y 반전. 나머지(XZ 평면)는 이름만 바꿈

PHI_M = M.PHI_M
PHI = {k: -v for k, v in M.PHI_M.items()}       # 최종 좌표의 게이트 좌우 각도


def mir(shape):
    """모델 → 최종 (Y 반전)."""
    return shape.mirror(Plane.XZ)


def mir2d(poly):
    return affinity.scale(poly, 1.0, -1.0, origin=(0, 0))


def _mdict(d):
    return {_SWAP.get(k, k): mir(v) for k, v in d.items()}


def build_all():
    st, cr, hb, fo, p2, camH = M.build_all()
    p2f = {}
    for k, v in p2.items():
        p2f[_SWAP.get(k, k)] = mir2d(v) if k in _XY_PLATES else v
    return _mdict(st), _mdict(cr), _mdict(hb), _mdict(fo), p2f, camH


# 바깥 스크립트가 직접 부르는 형상 함수 → 최종 좌표로
def make_seq_lock(seq=False):
    return mir(M.make_seq_lock(seq))


def make_mode_shaft(seq=False):
    return mir(M.make_mode_shaft(seq))


def make_mode_knob(seq=False):
    return mir(M.make_mode_knob(seq))


def make_boot():
    return mir(M.make_boot())


def desk_ghost(desk_t=M.DESK_T):
    return mir(M.desk_ghost(desk_t))


def make_clamp_pad(desk_t=M.DESK_T):
    return mir(M.make_clamp_pad(desk_t))


def make_clamp_bolt(desk_t=M.DESK_T):
    return mir(M.make_clamp_bolt(desk_t))


def make_clamp_knob(desk_t=M.DESK_T):
    return mir(M.make_clamp_knob(desk_t))


def gate_opening(clear=0.5):
    return mir2d(M.gate_opening(clear))


def extra_parts():
    """export/fusion_pack용 낱개 부품 (편심, 암 판 1장) — 최종 좌표."""
    ecc = M.cyl_x(5.5, -2, 6, y=M.SHAFT_Y, z=M.SHAFT_Z - 3) - M.cyl_x(2.55, -3, 7, y=M.SHAFT_Y, z=M.SHAFT_Z)
    arm_plate = M.plate_xz(M.arm2d(), M.YC + 8.1, M.YC + 11.1)
    return {"eccentric": mir(ecc), "arm_plate": mir(arm_plate)}


pose = M.pose                                  # 회전 공식은 좌표 반전과 무관 (최종 형상 + 최종 PHI로 그대로 씀)
