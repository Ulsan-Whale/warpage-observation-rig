
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class WarpageRigSimulatorApp:
    """
    다층 시편(Si/Cu) 열변형 관찰 장비 디지털 트윈(Digital Twin) 제어 및 섀도 모아레 시뮬레이션 GUI
    - Rev.B/C 열용량 모델(C = 179 J/K) 기반 2루프 온도 제어(블록 PID + 시편 CH3 외부 루프 보상)
    - 안전 인터록(Interlock) 및 에러 핸들링(과온 175°C 차단, 센서 단선, 도어 열림, 45°C 냉각 잠금)
    - 섀도 모아레(Shadow Moire) 4단계 위상 이동(p = 0.25 mm) 및 직교 측면 A/B 휨(δh) 실시간 시뮬레이션
    """

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Si/Cu 열변형 관찰 장비 — 정밀 온도 제어 & 섀도 모아레 시뮬레이션 GUI")
        self.root.geometry("1380x820")
        self.root.configure(bg="#F1F5F9")

        # 물리 및 장비 상수 (보고서 Rev.B/C 기준)
        self.T_amb = 25.0
        self.T_enc = 54.0
        self.C_block = 179.0          # 가열 모듈 총 열용량 (J/K)
        self.P_max = 100.0            # 카트리지 히터 최대 출력 (50W x 2)
        self.kappa_per_K = 4.2565e-3 / 120.8  # SimScale 보정 곡률 계수 (/mm/K)
        self.pitch = 0.25             # Ronchi 격자 피치 p (mm)

        # 제어 및 시뮬레이션 상태 변수
        self.running = False
        self.sim_time = 0.0           # 초 (s)
        self.dt = 1.0                 # 1 스텝 = 1초 (가속 배율 적용 가능)
        self.T_block = 25.0           # CH1: 블록 온도 (°C)
        self.T_spec = 25.0            # CH3: 시편 온도 (°C)
        self.T_door = 25.0            # CH4/CH5: 접근면 온도 (°C)
        self.pid_integral = 0.0
        self.prev_error = 0.0
        self.heater_power = 0.0

        # 안전 및 인터록 상태
        self.interlock_tripped = False
        self.trip_reason = "정상 (NORMAL)"
        self.door_locked = False

        # 데이터 로그
        self.hist_t = [0.0]
        self.hist_block = [25.0]
        self.hist_spec = [25.0]
        self.hist_target = [150.0]

        self._build_ui()
        self._update_optical_plots()

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Header.TLabel", font=("Malgun Gothic", 13, "bold"), background="#142C46", foreground="white")
        style.configure("Card.TLabelframe", background="#FFFFFF", foreground="#142C46", font=("Malgun Gothic", 10, "bold"))
        style.configure("Card.TLabelframe.Label", background="#FFFFFF", foreground="#142C46", font=("Malgun Gothic", 10, "bold"))
        style.configure("TLabel", background="#FFFFFF", foreground="#1E293B", font=("Malgun Gothic", 9))
        style.configure("Status.TLabel", background="#FFFFFF", foreground="#0F766E", font=("Malgun Gothic", 10, "bold"))

        # 상단 헤더 바
        header = tk.Frame(self.root, bg="#142C46", height=48)
        header.pack(fill=tk.X, side=tk.TOP)
        tk.Label(
            header,
            text="  Si/Cu 이중층 열변형 관찰 장비 (Warpage Observation Rig) — 디지털 트윈 제어 GUI",
            bg="#142C46",
            fg="white",
            font=("Malgun Gothic", 12, "bold"),
        ).pack(side=tk.LEFT, pady=10)

        main_frame = tk.Frame(self.root, bg="#F1F5F9")
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 좌측 제어 패널
        left_panel = tk.Frame(main_frame, bg="#F1F5F9", width=360)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_panel.pack_propagate(False)

        # 1. 공정 및 제어 설정 카드
        ctrl_card = ttk.LabelFrame(left_panel, text=" 1. 온도 프로파일 및 2루프 제어 설정 ", style="Card.TLabelframe", padding=10)
        ctrl_card.pack(fill=tk.X, pady=(0, 8))

        self.var_target_temp = tk.DoubleVar(value=150.0)
        self.var_ramp_rate = tk.DoubleVar(value=2.0)
        self.var_block_limit = tk.DoubleVar(value=175.0)
        self.var_speed = tk.IntVar(value=20)
        self.var_ch3_comp = tk.BooleanVar(value=True)
        self.var_liftoff = tk.BooleanVar(value=True)

        self._add_slider_row(ctrl_card, "목표 시편 온도 (°C):", self.var_target_temp, 30.0, 260.0, 0)
        self._add_slider_row(ctrl_card, "승온 램프 제한 (°C/min):", self.var_ramp_rate, 0.5, 10.0, 1)
        self._add_slider_row(ctrl_card, "블록 설정 상한 (°C):", self.var_block_limit, 150.0, 300.0, 2)
        self._add_slider_row(ctrl_card, "시뮬레이션 배속 (×):", self.var_speed, 1, 60, 3)

        ttk.Checkbutton(
            ctrl_card,
            text="시편 열전대(CH3) 외부 피드백 보상 활성화",
            variable=self.var_ch3_comp,
        ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(6, 2))

        ttk.Checkbutton(
            ctrl_card,
            text="평행 지지대 열적 들뜸(Lift-off) 접촉 열저항 반영",
            variable=self.var_liftoff,
        ).grid(row=5, column=0, columnspan=3, sticky="w", pady=(2, 4))

        # 실행 버튼
        btn_row = tk.Frame(ctrl_card, bg="#FFFFFF")
        btn_row.grid(row=6, column=0, columnspan=3, pady=(6, 0), sticky="ew")
        tk.Button(btn_row, text="▶ 가열 시작", bg="#0F766E", fg="white", font=("Malgun Gothic", 9, "bold"),
                  command=self.start_sim, width=10).pack(side=tk.LEFT, padx=2)
        tk.Button(btn_row, text="⏸ 일시정지", bg="#475569", fg="white", font=("Malgun Gothic", 9, "bold"),
                  command=self.pause_sim, width=9).pack(side=tk.LEFT, padx=2)
        tk.Button(btn_row, text="↺ 초기화", bg="#1E293B", fg="white", font=("Malgun Gothic", 9, "bold"),
                  command=self.reset_sim, width=8).pack(side=tk.LEFT, padx=2)

        # 2. 안전 인터록 및 에러 핸들링 테스트 카드
        safe_card = ttk.LabelFrame(left_panel, text=" 2. 안전 인터록 & 고장 모사 (Error Handling) ", style="Card.TLabelframe", padding=10)
        safe_card.pack(fill=tk.X, pady=(0, 8))

        self.var_door_closed = tk.BooleanVar(value=True)
        self.var_sensor_fault = tk.BooleanVar(value=False)

        ttk.Checkbutton(
            safe_card,
            text="전면 교체 도어 닫힘 (Door Interlock 정상)",
            variable=self.var_door_closed,
            command=self._check_safety_immediately,
        ).pack(anchor="w", pady=2)

        ttk.Checkbutton(
            safe_card,
            text="센서 단선/오픈 고장 모사 (Fail-Safe 차단)",
            variable=self.var_sensor_fault,
            command=self._check_safety_immediately,
        ).pack(anchor="w", pady=2)

        err_btn_row = tk.Frame(safe_card, bg="#FFFFFF")
        err_btn_row.pack(fill=tk.X, pady=(6, 2))
        tk.Button(err_btn_row, text="🛑 비상정지 (E-STOP)", bg="#DC2626", fg="white", font=("Malgun Gothic", 9, "bold"),
                  command=self.trigger_estop).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 3))
        tk.Button(err_btn_row, text="✔ 인터록 수동 복귀", bg="#2563EB", fg="white", font=("Malgun Gothic", 9, "bold"),
                  command=self.reset_interlock).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(3, 0))

        self.lbl_safety = tk.Label(
            safe_card,
            text="상태: 정상 운전 대기 (도어 잠금 해제)",
            bg="#ECFDF5",
            fg="#065F46",
            font=("Malgun Gothic", 9, "bold"),
            relief=tk.GROOVE,
            pady=5,
        )
        self.lbl_safety.pack(fill=tk.X, pady=(6, 0))

        # 3. 광학 계측 및 위상 이동 설정 카드
        opt_card = ttk.LabelFrame(left_panel, text=" 3. 섀도 모아레(C) & 측면 카메라(A·B) 설정 ", style="Card.TLabelframe", padding=10)
        opt_card.pack(fill=tk.X, pady=(0, 8))

        self.var_phase_step = tk.IntVar(value=0)
        self.var_tc_force = tk.DoubleVar(value=0.0)

        tk.Label(opt_card, text="4단계 위상 이동 (Z 스텝, p/4 = 62.5 μm):", bg="#FFFFFF", font=("Malgun Gothic", 9)).pack(anchor="w")
        phase_frame = tk.Frame(opt_card, bg="#FFFFFF")
        phase_frame.pack(fill=tk.X, pady=(2, 6))
        for step_idx, label_txt in enumerate(["0 (0°)", "1 (90°)", "2 (180°)", "3 (270°)"]):
            ttk.Radiobutton(
                phase_frame,
                text=label_txt,
                variable=self.var_phase_step,
                value=step_idx,
                command=self._update_optical_plots,
            ).pack(side=tk.LEFT, padx=3)

        f_row = tk.Frame(opt_card, bg="#FFFFFF")
        f_row.pack(fill=tk.X, pady=2)
        tk.Label(f_row, text="+Y 열전대 접촉력 (mN):", bg="#FFFFFF", font=("Malgun Gothic", 9)).pack(side=tk.LEFT)
        tk.Scale(
            f_row, from_=0.0, to=15.0, resolution=0.5, orient=tk.HORIZONTAL,
            variable=self.var_tc_force, bg="#FFFFFF", highlightthickness=0, length=150,
            command=lambda _: self._update_optical_plots()
        ).pack(side=tk.RIGHT)

        # 4. 실시간 계측 KPI 요약 카드
        kpi_card = ttk.LabelFrame(left_panel, text=" 4. 실시간 계측 및 KPI 판정 모니터 ", style="Card.TLabelframe", padding=10)
        kpi_card.pack(fill=tk.BOTH, expand=True)

        self.lbl_metrics = tk.Label(
            kpi_card,
            text="",
            justify=tk.LEFT,
            anchor="nw",
            bg="#FFFFFF",
            fg="#0F172A",
            font=("Consolas", 9.5),
        )
        self.lbl_metrics.pack(fill=tk.BOTH, expand=True)

        # 우측 실시간 그래프 영역 (Matplotlib 2x2)
        right_panel = tk.Frame(main_frame, bg="#FFFFFF", relief=tk.RIDGE, bd=1)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(9.5, 7.5), dpi=100)
        self.fig.subplots_adjust(left=0.07, right=0.96, top=0.93, bottom=0.08, wspace=0.26, hspace=0.34)

        self.ax_temp = self.fig.add_subplot(2, 2, 1)
        self.ax_moire = self.fig.add_subplot(2, 2, 2)
        self.ax_3dmap = self.fig.add_subplot(2, 2, 3)
        self.ax_edge = self.fig.add_subplot(2, 2, 4)

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_panel)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._refresh_metrics_label(0.0, 0.0, 0.0)

    def _add_slider_row(self, parent, label_text, var, vmin, vmax, row_idx):
        tk.Label(parent, text=label_text, bg="#FFFFFF", font=("Malgun Gothic", 9)).grid(row=row_idx, column=0, sticky="w", pady=2)
        sc = tk.Scale(
            parent, from_=vmin, to=vmax, resolution=0.5 if isinstance(var, tk.DoubleVar) else 1,
            orient=tk.HORIZONTAL, variable=var, bg="#FFFFFF", highlightthickness=0, length=135
        )
        sc.grid(row=row_idx, column=1, sticky="e", padx=4)
        val_lbl = tk.Label(parent, textvariable=var, bg="#FFFFFF", width=5, font=("Consolas", 9, "bold"))
        val_lbl.grid(row=row_idx, column=2, sticky="e")

    def start_sim(self):
        if not self._check_safety_immediately():
            return
        if not self.running:
            self.running = True
            self._step_loop()

    def pause_sim(self):
        self.running = False

    def reset_sim(self):
        self.running = False
        self.sim_time = 0.0
        self.T_block = 25.0
        self.T_spec = 25.0
        self.T_door = 25.0
        self.pid_integral = 0.0
        self.prev_error = 0.0
        self.heater_power = 0.0
        self.interlock_tripped = False
        self.trip_reason = "정상 (NORMAL)"
        self.door_locked = False
        self.hist_t = [0.0]
        self.hist_block = [25.0]
        self.hist_spec = [25.0]
        self.hist_target = [self.var_target_temp.get()]
        self._check_safety_immediately()
        self._update_optical_plots()

    def trigger_estop(self):
        self.interlock_tripped = True
        self.trip_reason = "비상정지(E-STOP) 작동 — 히터 전원 차단"
        self.running = False
        self.heater_power = 0.0
        self._update_safety_banner()

    def reset_interlock(self):
        if not self.var_door_closed.get() or self.var_sensor_fault.get():
            messagebox.showwarning("복귀 불가", "도어 닫힘 및 센서 정상 상태를 먼저 확인하세요.")
            return
        if self.T_block > self.var_block_limit.get() + 10.0:
            messagebox.showwarning("복귀 불가", "블록 온도가 과온 기준 이하로 냉각된 후 복귀 가능합니다.")
            return
        self.interlock_tripped = False
        self.trip_reason = "정상 복귀 완료"
        self._update_safety_banner()

    def _check_safety_immediately(self) -> bool:
        if self.var_sensor_fault.get():
            self.interlock_tripped = True
            self.trip_reason = "센서 단선 감지(Fail-Safe) — 가열 금지 & 도어 잠금"
            self.running = False
            self.heater_power = 0.0
        elif not self.var_door_closed.get():
            self.interlock_tripped = True
            self.trip_reason = "전면 도어 열림 감지 — 인터록 히터 차단"
            self.running = False
            self.heater_power = 0.0
        elif self.T_block > self.var_block_limit.get() + 10.0:
            self.interlock_tripped = True
            self.trip_reason = f"독립 과온 제한기 작동 (블록 > {self.var_block_limit.get()+10.0:.0f}°C)"
            self.running = False
            self.heater_power = 0.0

        # 고온 도어 잠금 기준 (CH1 또는 접근면 >= 45°C 시 잠금 유지)
        self.door_locked = (self.T_block >= 45.0) or (self.T_door >= 45.0) or self.var_sensor_fault.get()
        self._update_safety_banner()
        return not self.interlock_tripped

    def _update_safety_banner(self):
        lock_txt = "도어 잠금(LOCKED)" if self.door_locked else "잠금 해제(UNLOCKED)"
        if self.interlock_tripped:
            self.lbl_safety.config(
                text=f"⚠ TRIP: {self.trip_reason}\n[{lock_txt}]",
                bg="#FEF2F2", fg="#B91C1C"
            )
        else:
            self.lbl_safety.config(
                text=f"✔ {self.trip_reason} | 히터 {self.heater_power:.1f} W\n[{lock_txt}]",
                bg="#ECFDF5" if not self.door_locked else "#FFFBEB",
                fg="#065F46" if not self.door_locked else "#92400E"
            )

    def _step_loop(self):
        if not self.running:
            return

        speed = self.var_speed.get()
        T_goal = self.var_target_temp.get()
        ramp_per_sec = self.var_ramp_rate.get() / 60.0
        block_cap = self.var_block_limit.get()

        for _ in range(speed):
            if not self._check_safety_immediately():
                break

            self.sim_time += self.dt

            # 1) 램프 제한이 적용된 현재 목표 온도
            ramped_goal = min(T_goal, self.T_amb + ramp_per_sec * self.sim_time)

            # 2) 외부 루프 (CH3 시편 센서 피드백 -> 블록 목표 온도 보정)
            if self.var_ch3_comp.get():
                # 들뜸 열저항에 의한 온도 강하를 CH3 오차 적분으로 보상
                spec_err = ramped_goal - self.T_spec
                block_setpoint = min(block_cap, ramped_goal + 1.35 * spec_err + 12.0 * max(0.0, (self.T_spec - 25.0) / 125.0))
            else:
                block_setpoint = min(block_cap, ramped_goal)

            # 3) 내부 루프 (블록 Pt100 PID 제어)
            err = block_setpoint - self.T_block
            self.pid_integral = np.clip(self.pid_integral + err * self.dt, -200.0, 200.0)
            deriv = (err - self.prev_error) / self.dt
            self.prev_error = err

            u_pid = 8.5 * err + 0.08 * self.pid_integral + 12.0 * deriv
            self.heater_power = float(np.clip(u_pid, 0.0, self.P_max))

            # 4) 열전달 물리 모델 (Rev.B 손계산 및 들뜸 접촉 열저항 반영)
            # 블록 열손실 계수 UA_loss ≈ 0.12 W/K (블록 155°C 시 약 15 W 손실)
            q_loss = 0.12 * (self.T_block - self.T_amb)
            dT_block = ((self.heater_power - q_loss) / self.C_block) * self.dt
            self.T_block += dT_block

            # 시편 열전달: 들뜸(Lift-off) 활성화 시 접촉 열저항 증가로 평형 온도 강하 반영
            if self.var_liftoff.get():
                # 휨이 커질수록 접촉 면적이 2점으로 축소되어 열전달 효율 감소 (150°C에서 약 0.895)
                warpage_ratio = np.clip((self.T_spec - 25.0) / 125.0, 0.0, 1.5)
                coupling_eff = 0.966 - 0.072 * min(1.0, warpage_ratio)
                tau_spec = 14.0  # 들뜸 시 열시정수 증가 (s)
            else:
                coupling_eff = 0.966  # 평탄 이상 접촉 (Fusion 145.8/150 비율)
                tau_spec = 6.0

            T_spec_eq = self.T_amb + coupling_eff * (self.T_block - self.T_amb)
            self.T_spec += ((T_spec_eq - self.T_spec) / tau_spec) * self.dt
            self.T_door = self.T_amb + 0.14 * (self.T_block - self.T_amb)

        self.hist_t.append(self.sim_time / 60.0)  # 분 단위 기록
        self.hist_block.append(self.T_block)
        self.hist_spec.append(self.T_spec)
        self.hist_target.append(T_goal)

        self._update_safety_banner()
        self._update_optical_plots()

        if self.running:
            self.root.after(250, self._step_loop)

    def _update_optical_plots(self):
        # 1) 온도 추종 그래프 (좌상단)
        self.ax_temp.clear()
        self.ax_temp.plot(self.hist_t, self.hist_target, "k--", lw=1.2, label="목표 시편 온도")
        self.ax_temp.plot(self.hist_t, self.hist_block, color="#DC2626", lw=1.8, label="블록 온도 (CH1)")
        self.ax_temp.plot(self.hist_t, self.hist_spec, color="#0F766E", lw=2.2, label="시편 온도 (CH3)")
        self.ax_temp.set_title("1. 2루프 온도 제어 및 들뜸 보상 응답", fontsize=10, fontweight="bold", color="#142C46")
        self.ax_temp.set_xlabel("경과 시간 (min)", fontsize=8.5)
        self.ax_temp.set_ylabel("온도 (°C)", fontsize=8.5)
        self.ax_temp.grid(True, alpha=0.3)
        self.ax_temp.legend(loc="lower right", fontsize=7.5)

        # 현재 시편 온도에 따른 Si/Cu 이중층 3D 형상 계산
        dT = max(0.0, self.T_spec - self.T_amb)
        kappa = self.kappa_per_K * dT  # 1/mm
        x = np.linspace(-15.0, 15.0, 121)
        y = np.linspace(-15.0, 15.0, 121)
        X, Y = np.meshgrid(x, y)

        # 순수 접시형 휨 w(x, y) (mm)
        w_pure = 0.5 * kappa * (X**2 + Y**2)
        W_total = float(w_pure.max() - w_pure.min())

        # +Y 열전대 접촉력에 의한 2점 접촉 기울기 (K_theta = 7.01 mN*m/rad)
        F_tc = self.var_tc_force.get()
        K_theta = 7.0071  # mN*m/rad
        theta_rad = (F_tc * 15e-3) / K_theta if dT > 1.0 else 0.0
        tilt_y = -np.tan(theta_rad) * Y
        z_raw = w_pure + tilt_y

        # 2) 상부 카메라 C: 섀도 모아레 무늬 (4단계 위상 이동 반영) (우상단)
        self.ax_moire.clear()
        phase_shift = self.var_phase_step.get() * (np.pi / 2.0)
        # α = 45°, β = 0° -> Δh = ΔN * p -> 위상 φ = 2π * z / p
        phi = (2.0 * np.pi * z_raw / self.pitch) + phase_shift
        intensity = 0.5 * (1.0 + np.cos(phi))
        self.ax_moire.imshow(intensity, extent=[-15, 15, -15, 15], origin="lower", cmap="gray", vmin=0, vmax=1)
        self.ax_moire.axvline(-11.2, color="#38BDF8", ls="--", lw=0.9, alpha=0.8)
        self.ax_moire.axvline(11.2, color="#38BDF8", ls="--", lw=0.9, alpha=0.8)
        fringe_order = W_total / self.pitch
        self.ax_moire.set_title(
            f"2. 상부 C 섀도 모아레 무늬 (Step {self.var_phase_step.get()}, ~{fringe_order:.2f}차수)",
            fontsize=10, fontweight="bold", color="#142C46"
        )
        self.ax_moire.set_xlabel("X (mm)", fontsize=8.5)
        self.ax_moire.set_ylabel("Y (mm)", fontsize=8.5)

        # 3) 기준 평면 보정 후 3D 높이 지도 (좌하단)
        self.ax_3dmap.clear()
        w_um = (w_pure - w_pure.mean()) * 1000.0
        cf = self.ax_3dmap.contourf(X, Y, w_um, levels=16, cmap="viridis")
        cs = self.ax_3dmap.contour(X, Y, w_um, levels=8, colors="white", linewidths=0.5, alpha=0.8)
        self.ax_3dmap.clabel(cs, fmt="%.0f", fontsize=7)
        self.ax_3dmap.set_title(f"3. 위상 복원 & 기울기 보정 높이 지도 (W = {W_total:.3f} mm)", fontsize=10, fontweight="bold", color="#142C46")
        self.ax_3dmap.set_xlabel("X (mm)", fontsize=8.5)
        self.ax_3dmap.set_ylabel("Y (mm)", fontsize=8.5)

        # 4) 직교 측면 카메라 A·B 가장자리 실루엣 비교 (우하단)
        self.ax_edge.clear()
        edge_A = z_raw[0, :] * 1000.0       # Y = -15 가장자리 (μm)
        edge_B = z_raw[:, -1] * 1000.0      # X = +15 가장자리 (μm, 기울기 포함)
        dh_A = abs(edge_A[60] - 0.5 * (edge_A[0] + edge_A[-1]))
        dh_B_corr = abs(edge_B[60] - 0.5 * (edge_B[0] + edge_B[-1]))
        tilt_B_um = abs(edge_B[-1] - edge_B[0])

        self.ax_edge.plot(x, edge_A, color="#142C46", lw=2.0, label=f"카메라 A (-Y 가장자리, δh={dh_A:.0f} μm)")
        self.ax_edge.plot(y, edge_B, color="#DC2626", lw=2.0, ls="--", label=f"카메라 B (+X 원시, 기울기={tilt_B_um:.0f} μm)")
        self.ax_edge.plot([y[0], y[-1]], [edge_B[0], edge_B[-1]], color="#DC2626", lw=1.0, ls=":", label="B 양 끝 기준선 (보정 기준)")
        self.ax_edge.set_title("4. 측면 카메라 A·B 가장자리 윤곽 및 기준선 보정", fontsize=10, fontweight="bold", color="#142C46")
        self.ax_edge.set_xlabel("가장자리 좌표 X / Y (mm)", fontsize=8.5)
        self.ax_edge.set_ylabel("높이 Z (μm)", fontsize=8.5)
        self.ax_edge.grid(True, alpha=0.3)
        self.ax_edge.legend(loc="upper center", fontsize=7.2)

        self.canvas.draw_idle()
        self._refresh_metrics_label(W_total, dh_A / 1000.0, tilt_B_um / 1000.0)

    def _refresh_metrics_label(self, W_mm, dh_mm, tilt_mm):
        # 이론 자유 휨 (목표 온도 기준) 대비 현재 휨 오차율 계산
        dT_goal = max(1e-6, self.var_target_temp.get() - self.T_amb)
        W_ideal_goal = 0.25 * (self.kappa_per_K * dT_goal) * (30.0**2)
        err_pct = ((W_mm - W_ideal_goal) / W_ideal_goal) * 100.0 if self.sim_time > 0 else 0.0
        kpi_ok = "충족 (≤10%)" if abs(err_pct) <= 10.0 else "주의/초과"

        txt = (
            f"경과 시간   : {self.sim_time/60.0:6.1f} min\n"
            f"블록 (CH1)  : {self.T_block:6.2f} °C\n"
            f"시편 (CH3)  : {self.T_spec:6.2f} °C\n"
            f"접근면 온도 : {self.T_door:6.2f} °C\n"
            f"------------------------------\n"
            f"전체 휨 (W) : {W_mm:6.4f} mm\n"
            f"측면 휨(δh) : {dh_mm:6.4f} mm\n"
            f"B 원시기울기: {tilt_mm:6.4f} mm\n"
            f"목표대비오차: {err_pct:+6.2f} % ({kpi_ok})"
        )
        self.lbl_metrics.config(text=txt)


if __name__ == "__main__":
    root = tk.Tk()
    app = WarpageRigSimulatorApp(root)
    root.mainloop()
