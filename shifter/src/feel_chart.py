# LSK H/SEQ Shifter — author: Sunggwan
import numpy as np, matplotlib
matplotlib.use("Agg"); matplotlib.rcParams["font.family"]="Noto Sans CJK JP"; matplotlib.rcParams["axes.unicode_minus"]=False
import matplotlib.pyplot as plt
import cam as C
cam = C.build_cam(C.H_PROFILE)
ths, psis, T, Fk = C.simulate(cam)
tgt = C.torque_fn(C.H_PROFILE)(ths)/(C.L_KNOB/1000)
fig, axs = plt.subplots(1,2, figsize=(12,4.6))
ax=axs[0]; ax.plot(ths,tgt,"--",label="목표"); ax.plot(ths,Fk,label="시뮬레이션"); ax.axhline(0,color="k",lw=.5)
for s in (-14,14): ax.axvline(s,color="gray",ls=":",lw=1)
ax.set_title("H 모드: 노브 힘 (레버 110mm, 14° 금속 스톱)"); ax.set_xlabel("앞뒤 각도 (°)  + = 앞"); ax.set_ylabel("노브 힘 (N)  + = 밀어내는 방향"); ax.legend(); ax.grid(alpha=.3)
ax=axs[1]; msk=np.abs(ths)<=8
ax.plot(ths[msk],Fk[msk],color="C1",label="시퀀셜 = H 캠의 ±8° 구간"); ax.axhline(0,color="k",lw=.5)
for s in (-8,8): ax.axvline(s,color="C3",lw=2)
ax.axvspan(6.4,8,color="C2",alpha=.12); ax.axvspan(-8,-6.4,color="C2",alpha=.12)
ax.annotate("클릭 (20N → 7N)", xy=(7.2,12), xytext=(8.6,16), arrowprops=dict(arrowstyle="->"))
ax.annotate("잠금 포켓 스톱 8°", xy=(8,2), xytext=(8.6,-8), arrowprops=dict(arrowstyle="->"))
ax.set_title("시퀀셜 모드: 힘이 항상 중앙 쪽 → 자동 복귀"); ax.set_xlabel("앞뒤 각도 (°)"); ax.set_ylabel("노브 힘 (N)"); ax.legend(loc="upper left"); ax.grid(alpha=.3)
ax.set_xlim(-14.5,16); ax.set_ylim(-24,24)
fig.tight_layout(); fig.savefig("out/feel_curves.png", dpi=125)
