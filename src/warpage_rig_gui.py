!apt-get -qq update && apt-get -qq install -y fonts-nanum
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

font_path = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
fm.fontManager.addfont(font_path)
plt.rc("font", family="NanumGothic")
plt.rcParams["axes.unicode_minus"] = False

import numpy as np
import matplotlib.pyplot as plt
import ipywidgets as widgets
from IPython.display import display, clear_output

# 물리 및 장비 상수
T_AMB = 25.0
C_BLOCK = 179.0  # 가열 모듈 열용량 (J/K)
P_MAX = 100.0    # 카트리지 히터 최대 출력 (W)
KAPPA_PER_K = 4.2565e-3 / 120.8
PITCH = 0.25     # 론키 격자(Ronchi Grating) 피치 (mm)
K_THETA = 7.0071 # 열전대 접촉 틸트 강성 (mN*m/rad)

def simulate_and_plot(target_temp, block_limit, ramp_rate, duration_min,
                      ch3_comp, liftoff, phase_step, tc_force):
    dt = 2.0
    steps = int((duration_min * 60.0) / dt)

    t_block = T_AMB
    t_spec = T_AMB
    integral_err = 0.0
    prev_err = 0.0

    t_hist = [0.0]
    b_hist = [t_block]
    s_hist = [t_spec]

    ramp_per_sec = ramp_rate / 60.0

    for s in range(1, steps + 1):
        sim_t = s * dt
        ramped_goal = min(target_temp, T_AMB + ramp_per_sec * sim_t)

        if ch3_comp:
            spec_err = ramped_goal - t_spec
            block_sp = min(
                block_limit,
                ramped_goal + 1.35 * spec_err + 12.0 * max(0.0, (t_spec - 25.0) / 125.0)
            )
        else:
            block_sp = min(block_limit, ramped_goal)

        err = block_sp - t_block
        integral_err = np.clip(integral_err + err * dt, -200.0, 200.0)
        deriv = (err - prev_err) / dt
        prev_err = err

        u_pid = 8.5 * err + 0.08 * integral_err + 12.0 * deriv
        heater_w = float(np.clip(u_pid, 0.0, P_MAX))

        q_loss = 0.12 * (t_block - T_AMB)
        t_block += ((heater_w - q_loss) / C_BLOCK) * dt

        if liftoff:
            w_ratio = np.clip((t_spec - 25.0) / 125.0, 0.0, 1.5)
            coupling_eff = 0.966 - 0.072 * min(1.0, w_ratio)
            tau_spec = 14.0
        else:
            coupling_eff = 0.966
            tau_spec = 6.0

        t_spec_eq = T_AMB + coupling_eff * (t_block - T_AMB)
        t_spec += ((t_spec_eq - t_spec) / tau_spec) * dt

        t_hist.append(sim_t / 60.0)
        b_hist.append(t_block)
        s_hist.append(t_spec)

    # 광학 계측 및 휨 계산
    dT = max(0.0, t_spec - T_AMB)
    kappa = KAPPA_PER_K * dT
    x = np.linspace(-15.0, 15.0, 121)
    y = np.linspace(-15.0, 15.0, 121)
    X, Y = np.meshgrid(x, y)

    w_pure = 0.5 * kappa * (X**2 + Y**2)
    W_total = float(w_pure.max() - w_pure.min())

    theta_rad = (tc_force * 15e-3) / K_THETA if dT > 1.0 else 0.0
    tilt_y = -np.tan(theta_rad) * Y
    z_raw = w_pure + tilt_y

    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5))
    plt.subplots_adjust(wspace=0.28, hspace=0.36)

    # 1. 온도 응답
    ax = axes[0, 0]
    ax.axhline(target_temp, color="k", ls="--", lw=1.1, label="목표 시편 온도")
    ax.plot(t_hist, b_hist, color="#DC2626", lw=1.8, label="블록 (CH1)")
    ax.plot(t_hist, s_hist, color="#0F766E", lw=2.2, label="시편 (CH3)")
    ax.set_title("1. 2루프 온도 제어 응답")
    ax.set_xlabel("시간 (min)")
    ax.set_ylabel("온도 (°C)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)

    # 2. 섀도 모아레 무늬
    ax = axes[0, 1]
    phi = (2.0 * np.pi * z_raw / PITCH) + phase_step * (np.pi / 2.0)
    intensity = 0.5 * (1.0 + np.cos(phi))
    ax.imshow(intensity, extent=[-15, 15, -15, 15], origin="lower", cmap="gray", vmin=0, vmax=1)
    ax.axvline(-11.2, color="#38BDF8", ls="--", lw=0.9)
    ax.axvline(11.2, color="#38BDF8", ls="--", lw=0.9)
    ax.set_title(f"2. 섀도 모아레 무늬 (Step {phase_step}, ~{W_total/PITCH:.2f}차수)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")

    # 3. 높이 지도
    ax = axes[1, 0]
    w_um = (w_pure - w_pure.mean()) * 1000.0
    cf = ax.contourf(X, Y, w_um, levels=16, cmap="viridis")
    plt.colorbar(cf, ax=ax, label="μm")
    ax.set_title(f"3. 보정 후 휨 높이 지도 (W = {W_total:.3f} mm)")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")

    # 4. 측면 가장자리 윤곽
    ax = axes[1, 1]
    edge_A = z_raw[0, :] * 1000.0
    edge_B = z_raw[:, -1] * 1000.0
    ax.plot(x, edge_A, color="#142C46", lw=2.0, label="카메라 A (-Y)")
    ax.plot(y, edge_B, color="#DC2626", lw=2.0, ls="--", label="카메라 B (+X 원시)")
    ax.set_title("4. 측면 카메라 A·B 가장자리 프로파일")
    ax.set_xlabel("좌표 (mm)")
    ax.set_ylabel("높이 Z (μm)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    plt.show()
    print(f"최종 블록 온도: {t_block:.2f} °C | 최종 시편 온도: {t_spec:.2f} °C | 순수 휨 W: {W_total:.4f} mm")

ui = widgets.interactive(
    simulate_and_plot,
    target_temp=widgets.FloatSlider(value=150.0, min=30.0, max=260.0, step=5.0, description="목표온도(°C)"),
    block_limit=widgets.FloatSlider(value=175.0, min=150.0, max=300.0, step=5.0, description="블록상한(°C)"),
    ramp_rate=widgets.FloatSlider(value=5.0, min=1.0, max=20.0, step=1.0, description="승온속도"),
    duration_min=widgets.IntSlider(value=15, min=2, max=60, step=1, description="해석시간(분)"),
    ch3_comp=widgets.Checkbox(value=True, description="CH3 피드백 보상"),
    liftoff=widgets.Checkbox(value=True, description="지지대 들뜸 반영"),
    phase_step=widgets.Dropdown(options=[("0 (0°)", 0), ("1 (90°)", 1), ("2 (180°)", 2), ("3 (270°)", 3)], value=0, description="위상스텝"),
    tc_force=widgets.FloatSlider(value=0.0, min=0.0, max=15.0, step=0.5, description="열전대힘(mN)"),
)
display(ui)
