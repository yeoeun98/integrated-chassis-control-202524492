# [202524492-yeoeun Oh] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄  
**제출일**: 2026-06-22  
**팀**: 개인

---

## 1. 설계 개요

본 프로젝트의 목표는 14-DOF 차량 모델에서 통합 섀시 제어기(Integrated Chassis Control)를 설계하여 조향 안정성, 제동 안정성, 승차감을 동시에 개선하는 것이다. 검증 plant는 14-DOF 모델까지 포함하지만, 제어기 설계는 강의에서 다룬 선형 bicycle model, PID/PI 제어, saturation, anti-windup, gain scheduling, skyhook damping을 기반으로 단순화하였다. 이는 실제 차량 제어에서도 상위 제어기는 단순 모델로 목표 yaw rate, slip angle, 제동 요구량을 계산하고, 하위 actuator allocation에서 물리 한계를 반영하는 방식이 일반적이기 때문이다.

본 설계에서는 LQR 대신 PID/PI 기반 제어를 선택하였다. LQR은 상태공간 모델과 가중치 행렬을 통해 다변수 최적 제어를 설계할 수 있다는 장점이 있지만, 본 과제의 채점 KPI는 yaw-rate step 응답, double lane change, brake-in-turn, straight braking처럼 시나리오별로 포화와 타이어 slip이 지배적인 구간이 많다. 따라서 선형 근방의 최적성보다 gain scheduling, deadband, saturation, ABS slip limiting을 직접 조절할 수 있는 PID/PI 구조가 KPI 기반 반복 튜닝에 더 적합하다고 판단하였다.

각 제어기의 역할은 다음과 같다.

| 모듈 | 설계 요약 |
|---|---|
| `ctrl_lateral` | yaw rate 오차에 대한 PD 기반 AFS 보조조향 + slip angle 기반 ESC yaw moment |
| `ctrl_longitudinal` | 속도 오차 PI 제어 + brake active 시 최대 안전 감속 + wheel slip 기반 ABS 토크 감소 |
| `ctrl_vertical` | 4륜 독립 on-off skyhook CDC |
| `ctrl_coordinator` | AFS 조향 통과, 60:40 전후 제동 분배, yaw moment를 좌우 차동 제동으로 변환 |

---

## 2. 수학적 모델링

### 2.1 사용한 plant 단순화

제어기 설계에는 2-DOF linear bicycle model을 사용하였다. 실제 시뮬레이션은 `plant_14dof.m`에서 종방향, 횡방향, yaw, roll, pitch, wheel dynamics, suspension dynamics까지 포함하지만, 횡방향 제어기 설계 단계에서는 상태를 횡속도 `v_y`와 yaw rate `r`로 줄였다. 이 모델은 조향 입력과 yaw rate 응답 사이의 관계를 명확히 보여주기 때문에 AFS의 yaw rate 추종 제어와 ESC 개입 조건을 설계하기에 적합하다.

종방향 제어는 차량 전체 질량을 이용한 lumped mass model로 단순화하였다.

$$F_x = m a_x$$

제동 중에는 타이어 slip이 제동거리와 안정성에 큰 영향을 주므로, wheel slip ratio를 다음과 같이 보았다.

$$\kappa_i = \frac{\omega_i r_w - v_x}{\max(|v_x|, 0.1)}$$

수직 제어는 quarter-car 개념의 sprung/unsprung 상대속도만 이용하였다. `ctrl_vertical`은 각 휠의 sprung velocity `z_s_dot`와 suspension relative velocity `z_s_dot - z_u_dot`의 부호를 기준으로 감쇠 계수를 switching한다.

### 2.2 State-space 표현

횡방향 제어 설계에 사용한 bicycle model의 상태, 입력, 출력은 다음과 같다.

$$x = \begin{bmatrix} v_y \\ r \end{bmatrix}, \quad u = \delta, \quad y = \begin{bmatrix} v_y \\ r \end{bmatrix}$$

$$\dot{x} = Ax + Bu, \quad y = Cx + Du$$

차량 파라미터는 `config/sim_params.m`의 기본값을 기준으로 하였다.

| 기호 | 의미 | 값 |
|---|---:|---:|
| `m` | 차량 질량 | 1500 kg |
| `I_z` | yaw 관성모멘트 | 2500 kg m² |
| `l_f` | CG-전축 거리 | 1.2 m |
| `l_r` | CG-후축 거리 | 1.4 m |
| `C_f` | 전륜 코너링 강성 | 80000 N/rad |
| `C_r` | 후륜 코너링 강성 | 85000 N/rad |

상태방정식은 다음과 같다.

$$
\dot{v}_y =
-\frac{C_f + C_r}{mV_x}v_y
+ \left(\frac{l_r C_r - l_f C_f}{mV_x} - V_x\right)r
+ \frac{C_f}{m}\delta
$$

$$
\dot{r} =
\frac{l_r C_r - l_f C_f}{I_zV_x}v_y
- \frac{l_f^2 C_f + l_r^2 C_r}{I_zV_x}r
+ \frac{l_f C_f}{I_z}\delta
$$

따라서

$$
A =
\begin{bmatrix}
-\frac{C_f+C_r}{mV_x} &
\frac{l_rC_r-l_fC_f}{mV_x}-V_x \\
\frac{l_rC_r-l_fC_f}{I_zV_x} &
-\frac{l_f^2C_f+l_r^2C_r}{I_zV_x}
\end{bmatrix},
\quad
B =
\begin{bmatrix}
\frac{C_f}{m} \\
\frac{l_fC_f}{I_z}
\end{bmatrix}
$$

목표 yaw rate는 정상상태 bicycle model을 이용하여 계산하였다.

$$
r_{ref} = \frac{V_x \delta}{L + K_{us}V_x^2}
$$

여기서

$$
K_{us}
= \frac{m l_r}{2C_fL}
- \frac{m l_f}{2C_rL}
$$

이 식은 `calc_ref_yaw_rate.m`에 구현되어 있으며, 저속 영역에서는 yaw rate reference를 0으로 제한하여 수치적으로 불안정한 목표값을 피했다.

### 2.3 가정과 한계

첫째, 제어기 설계 단계에서는 종방향 속도 `V_x`를 순간적으로 일정하다고 가정하였다. 실제 14-DOF plant에서는 제동과 조향이 동시에 발생하며 속도가 변하지만, 상위 yaw rate reference와 PD 보상기는 매 step 현재 `v_x`를 받아 gain scheduling을 적용하므로 이 가정을 부분적으로 보완한다.

둘째, bicycle model은 선형 타이어를 가정한다. A7 brake-in-turn이나 D1 DLC+brake처럼 큰 slip이 발생하는 시나리오에서는 선형 모델의 정확도가 낮아진다. 이를 보완하기 위해 slip angle threshold 기반 ESC yaw moment와 wheel slip 기반 ABS를 별도로 추가하였다.

셋째, 본 설계는 actuator saturation을 적극적으로 사용한다. 보조조향은 ±2 deg 수준으로 제한하고, yaw moment는 ±1000 Nm에서 제한하였다. 이는 과도한 제어 입력이 오히려 LTR, lateral deviation, baseline 대비 KPI를 악화시키는 것을 막기 위한 보수적 선택이다.

---

## 3. 제어기 설계

### 3.1 `ctrl_lateral` — AFS + ESC

#### 설계 목표

횡방향 제어기의 목표는 다음과 같다.

| 목표 | 구현 방식 |
|---|---|
| yaw rate reference 추종 | yaw error 기반 PD 보조조향 |
| 과도한 sideslip 억제 | slip angle threshold 초과 시 ESC yaw moment |
| 고속 안정성 확보 | 속도 기반 gain scheduling 및 조향 제한 |
| windup 방지 | 적분항 제거 및 deadband 적용 |

AFS 보조조향은 다음 yaw rate error를 사용한다.

$$e_r = r_{ref} - r$$

초기 템플릿에는 PID 형태가 제시되어 있었지만, 최종 구현에서는 적분항을 제거하였다.

```matlab
gainScale = 1 + 0.15 * exp(-vx_safe / 10);
Kp = CTRL.LAT.Kp * gainScale * 0.7;
Ki = 0;
Kd = CTRL.LAT.Kd * 0.4;
```

최종 보조조향 명령은 다음과 같이 표현할 수 있다.

$$
\delta_{AFS}
= sat_{\pm 2deg}
\left(K_p e_r + K_d \dot{e}_{r,f}\right)
$$

여기서 `dot(e_r,f)`는 1차 저역통과 필터를 통과한 derivative이다.

```matlab
dRaw = (yawErr - ctrlState.prevError) / max(dt, eps);
alpha = 0.7;
ctrlState.dFilt = alpha * ctrlState.dFilt + (1-alpha) * dRaw;
```

derivative filter를 둔 이유는 step steer나 급격한 driver input에서 derivative kick이 보조조향을 순간적으로 크게 만들 수 있기 때문이다. 또한 yaw error가 0.5 deg/s 미만이고 slip angle이 0.8 deg 미만이면 보조조향을 0으로 두어 작은 오차에 과도하게 반응하지 않도록 했다.

ESC는 slip angle이 속도별 threshold를 초과할 때만 개입한다.

```matlab
beta_th = min(LIM.MAX_SLIP_ANGLE, deg2rad(4.0 + min(vx_safe / 20, 1.0) * 1.0));
Kbeta = 550;
speedFactor = min(max(vx_safe / 15, 0.2), 1.0);
yawMoment = -Kbeta * sign(slipAngle) * (abs(slipAngle) - beta_th) * speedFactor;
yawMoment = max(-1000, min(1000, yawMoment));
```

수식으로 쓰면 다음과 같다.

$$
M_z =
\begin{cases}
-K_\beta \, sign(\beta)(|\beta|-\beta_{th}) f(V_x), & |\beta| > \beta_{th} \\
0, & |\beta| \le \beta_{th}
\end{cases}
$$

$$
f(V_x)=clip(V_x/15, 0.2, 1.0)
$$

#### 왜 PID/PD를 선택했는가

LQR은 `x=[v_y,r]^T` 상태를 모두 feedback할 수 있어 이론적으로 더 체계적이다. 그러나 본 과제의 입력은 reference yaw rate, 실제 yaw rate, slip angle이 직접 주어지는 형태이고, KPI는 overshoot, rise time, settling, side slip, LTR처럼 해석 가능한 시간응답 지표이다. 따라서 PD 보조조향과 slip threshold ESC를 분리하면 각 KPI에 대한 영향이 명확하고, 조향 한계와 yaw moment 한계를 직접 둘 수 있다. 또한 grading rule이 baseline보다 나빠진 KPI를 0점 처리하기 때문에, 공격적인 LQR보다 보수적인 assist-only PD가 더 안정적인 점수 확보에 유리했다.

#### Gain 산정 과정

초기값은 `sim_params.m`의 `CTRL.LAT.Kp=1.0`, `CTRL.LAT.Kd=0.05`를 사용하였다. 이후 A3 step steer에서 overshoot, rise time, settling time을 기준으로 반복 조정하였다.

1. 초기 PID 구조에서는 적분항이 yaw rate 정상상태 오차를 줄일 수 있지만, DLC와 brake-in-turn에서 slip angle이 커질 때 windup으로 보조조향이 늦게 풀리는 문제가 있었다.
2. 따라서 `Ki=0`으로 제거하고 PD로 단순화하였다.
3. `Kp`는 기본값의 70%로 낮추어 AFS를 driver 조향을 대체하는 제어가 아니라 보조 제어로 제한하였다.
4. `Kd`는 기본값의 40%로 낮추고 derivative filter를 추가하여 step 입력에서의 kick을 줄였다.
5. ESC threshold는 3 deg보다 약간 높은 4~5 deg로 설정하였다. 지나치게 낮은 threshold는 A1/D1에서 불필요한 좌우 제동을 만들고 LTR과 path deviation을 악화시켰기 때문이다.

최종 게인은 다음과 같다.

| 항목 | 최종값 |
|---|---:|
| `CTRL.LAT.Kp` | 1.0 |
| 실제 `Kp` | `CTRL.LAT.Kp * gainScale * 0.7` |
| `CTRL.LAT.Ki` | 사용하지 않음 |
| `CTRL.LAT.Kd` | 0.05 |
| 실제 `Kd` | `CTRL.LAT.Kd * 0.4` |
| yaw deadband | 0.5 deg/s |
| slip deadband | 0.8 deg |
| steering saturation | ±2.0 deg |
| `Kbeta` | 550 |
| ESC yaw moment saturation | ±1000 Nm |

### 3.2 `ctrl_longitudinal` — 속도 제어 + ABS

종방향 제어기는 일반 주행에서는 PI 속도 제어를 사용하고, 시나리오가 강제 제동 상태(`brakeActive=true`)일 때는 속도 추종보다 제동 안정성과 제동거리 단축을 우선한다.

속도 오차는 다음과 같다.

$$e_v = V_{x,ref} - V_x$$

비제동 상태의 PI 제어식은 다음과 같다.

$$F_x = K_p e_v + K_i \int e_v dt$$

코드에서는 다음 게인을 사용한다.

| 항목 | 값 |
|---|---:|
| `CTRL.LON.Kp` | 2.0 |
| `CTRL.LON.Ki` | 0.12 |
| `CTRL.LON.intMax` | 2000 |

제동 상태에서는 적분기를 reset하고 최대 안전 감속을 목표로 한다.

```matlab
targetDecel = -min(0.95 * g, LIM.MAX_AX);
Fx_cmd = mass * targetDecel;
```

즉,

$$
F_{x,cmd}=m \cdot \left[-\min(0.95g, a_{x,max})\right]
$$

다만 실제 명령은 jerk limit을 통과한다.

```matlab
maxDeltaF = 3 * LIM.MAX_JERK * mass * dt;
Fx_cmd = max(ctrlState.prevForce - maxDeltaF, ...
             min(ctrlState.prevForce + maxDeltaF, Fx_cmd));
```

ABS는 wheel slip ratio가 0.12를 초과할 때 작동한다.

```matlab
slipLimit = 0.12;
slipExcess = max(abs(prevSlipRatio) - slipLimit, 0);
scale = min(0.25, max(0.03, max(slipExcess) / 0.30));
forceCmd.brakeOffset = -scale * 0.6 * LIM.MAX_BRAKE_TRQ * brakeShare;
```

이 방식은 bang-bang ABS보다 부드럽다. 과도 slip 휠의 제동 토크를 줄이되, 전체 제동력을 너무 크게 잃지 않도록 최대 감소율을 제한하였다. 그 결과 B1 straight brake에서 stopping distance와 abs slip RMS를 모두 만족하였다.

### 3.3 `ctrl_vertical` — CDC

수직 제어기는 4륜 독립 on-off skyhook CDC를 사용하였다. 각 휠에 대해 sprung mass 속도와 damper relative velocity의 곱을 계산한다.

$$
s_i = \dot{z}_{s,i}(\dot{z}_{s,i}-\dot{z}_{u,i})
$$

조건은 다음과 같다.

$$
c_i =
\begin{cases}
c_{max}, & s_i > 0 \\
c_{min}, & s_i \le 0
\end{cases}
$$

코드상 값은 다음과 같다.

| 항목 | 값 |
|---|---:|
| `CTRL.VER.cMin` | 500 Ns/m |
| `CTRL.VER.cMax` | 5000 Ns/m |
| `CTRL.VER.skyGain` | 2500 Ns/m |

skyhook 제어는 sprung mass가 절대좌표계에서 흔들리지 않도록 만드는 개념이다. Semi-active damper는 에너지를 능동적으로 주입할 수 없으므로, damper가 body motion을 줄이는 방향으로 작용할 때만 큰 감쇠를 사용하고 그렇지 않을 때는 작은 감쇠를 사용하였다.

### 3.4 `ctrl_coordinator` — Actuator Allocation

통합 조율기는 상위 제어기의 조향, 제동, 감쇠 명령을 실제 actuator 명령으로 변환한다.

#### 조향 배분

AFS 보조조향은 steering limit 내에서 그대로 통과한다.

$$
\delta_{cmd}=clip(\delta_{AFS},-\delta_{max},\delta_{max})
$$

#### 종방향 제동 배분

종방향 힘 명령이 음수이면 휠 토크로 변환한다.

$$
T_{total} = \frac{|F_x| r_w}{4}
$$

이후 전후 60:40으로 분배한다.

$$
T_f = 0.6T_{total}, \quad T_r=0.4T_{total}
$$

$$
T_{FL}=T_{FR}=T_f/2, \quad T_{RL}=T_{RR}=T_r/2
$$

#### ESC yaw moment 배분

ESC yaw moment는 전후 50:50으로 나누고, 각 axle에서 좌우 차동 제동으로 만든다.

$$
M_{z,f}=0.5M_z, \quad M_{z,r}=0.5M_z
$$

$$
\Delta_f = \frac{M_{z,f}}{t_f/2}, \quad
\Delta_r = \frac{M_{z,r}}{t_r/2}
$$

코드에서는 다음과 같이 좌우 토크를 조정한다.

```matlab
brakeFL = brakeFL + delta_f / 2;
brakeFR = brakeFR - delta_f / 2;
brakeRL = brakeRL + delta_r / 2;
brakeRR = brakeRR - delta_r / 2;
```

추가로 yaw moment가 현재 base brake torque보다 커서 음수 휠 토크를 만들지 않도록 가능한 yaw moment를 제한하였다. 이 제한을 둔 이유는, 음수 토크가 saturation에서 잘리면 의도한 yaw moment가 왜곡되고 전체 제동력이 급격히 변할 수 있기 때문이다.

---

## 4. 시뮬레이션 결과

### 4.1 Auto grading 결과 요약

현재 `grade_report.json` 기준 결과는 다음과 같다. 단, 이 파일은 `student_info.m` 수정 전 생성된 결과이므로, 최종 제출 전 `grade.m`을 다시 실행하면 학생 정보 미기입 감점은 제거될 것으로 예상된다.

| 항목 | 점수 |
|---|---:|
| 정량 점수 | 62.205 / 70 |
| 정량 달성률 | 88.865% |
| 기존 감점 | -5 |
| 기존 최종 자동 점수 | 57.205 |
| `student_info.m` 수정 후 예상 최종 점수 | 62.205 |

기존 감점 사유는 `student_info.m` 미기입이었다. 현재 `student_info.m`에는 학번, 이름, AI 사용 내역이 작성되어 있으므로 제출 전 `grade.m`을 다시 실행해 `grade_report.json`을 갱신하는 것이 좋다. 제어 성능 자체의 정량 점수는 62점대로, 대부분의 핵심 KPI를 만족하였다.

### 4.2 KPI별 결과

| 시나리오 | KPI | 결과값 | 목표값 | 점수 |
|---|---|---:|---:|---:|
| A3 step steer | yawRateOvershoot [%] | 2.599 | 10 | 4 / 4 |
| A3 step steer | yawRateRiseTime [s] | 0.227 | 0.3 | 4 / 4 |
| A3 step steer | yawRateSettling [s] | 0.744 | 0.8 | 4 / 4 |
| A1 DLC | sideSlipMax [deg] | 2.837 | 3 | 6 / 6 |
| A1 DLC | LTR_max | 0.744 | 0.6 | 3.801 / 5 |
| A1 DLC | lateralDevMax [m] | 1.866 | 0.7 | 0 / 4 |
| A4 steady state | understeerGradient | 0.000748 | 0.003 | 5 / 5 |
| A4 steady state | sideSlipMax [deg] | 1.179 | 2 | 5 / 5 |
| A7 brake in turn | sideSlipMax [deg] | 2.950 | 5 | 8 / 8 |
| A7 brake in turn | LTR_max | 0.198 | 0.7 | 7 / 7 |
| B1 straight brake | stoppingDistance [m] | 38.825 | 40 | 5 / 5 |
| B1 straight brake | absSlipRMS | 0.0925 | 0.1 | 5 / 5 |
| D1 DLC + brake | sideSlipMax [deg] | 4.117 | 4 | 3.883 / 4 |
| D1 DLC + brake | LTR_max | 0.744 | 0.6 | 1.521 / 2 |
| D1 DLC + brake | lateralDevMax [m] | 1.866 | 1 | 0 / 2 |

### 4.3 A3 step steer 분석

A3에서는 yaw rate overshoot, rise time, settling time이 모두 만점이다. 이는 PD 기반 AFS가 yaw rate error에 빠르게 반응하면서도, 조향각을 ±2 deg로 제한하고 derivative filter를 적용하여 overshoot를 억제했기 때문이다. 적분항을 제거한 것도 step 응답에서 windup을 줄이는 데 기여하였다.

결과적으로 yawRateOvershoot는 2.599%로 목표 10%보다 충분히 낮고, yawRateRiseTime은 0.227 s로 목표 0.3 s 이내이며, yawRateSettling은 0.744 s로 목표 0.8 s를 만족하였다.

### 4.4 A1 double lane change 분석

A1에서는 sideSlipMax가 2.837 deg로 목표 3 deg를 만족하여 만점을 받았다. 이는 slip angle이 threshold를 넘을 때만 ESC yaw moment를 걸어 sideslip을 억제한 결과이다. 반면 LTR_max는 0.744로 목표 0.6보다 커 일부 감점되었고, lateralDevMax는 1.866 m로 목표 0.7 m를 넘어서 0점이 되었다.

이 결과는 현재 제어기가 안정성 중심으로 튜닝되어 있음을 보여준다. 즉, 차량이 크게 미끄러지는 것은 막았지만 reference path를 정밀하게 따라가는 기능은 충분하지 않다. AFS 보조조향을 보수적으로 제한했기 때문에 side slip은 줄었지만 path tracking error를 적극적으로 줄이지는 못했다.

### 4.5 A7 brake-in-turn 분석

A7은 가장 성공적인 시나리오 중 하나이다. sideSlipMax는 2.950 deg로 목표 5 deg보다 낮고, LTR_max는 0.198로 목표 0.7보다 매우 낮아 두 KPI 모두 만점을 받았다.

이 시나리오에서는 종방향 제동과 횡방향 안정성이 동시에 중요하다. 현재 설계는 제동 중 PI 속도 추종을 끄고 최대 안전 감속을 사용하며, wheel slip이 커질 때 휠별 brake offset을 적용한다. 동시에 lateral controller의 ESC yaw moment가 slip angle을 억제한다. 이 조합으로 회전 중 급제동에서도 스핀아웃을 방지하였다.

### 4.6 B1 straight brake 분석

B1에서는 stoppingDistance가 38.825 m로 목표 40 m를 만족했고, absSlipRMS는 0.0925로 목표 0.1보다 낮아 두 KPI 모두 만점을 받았다. 이는 `targetDecel = -min(0.95g, LIM.MAX_AX)`로 충분한 제동력을 확보하면서도, slip ratio가 0.12를 넘는 휠에만 제한적으로 제동 토크를 줄이는 ABS가 효과적으로 작동했기 때문이다.

강한 제동력을 유지하면서 slip RMS를 낮게 유지한 점이 핵심이다. ABS 감쇠율을 너무 크게 두면 slip은 줄지만 stopping distance가 길어지고, 너무 작게 두면 stopping distance는 짧아도 slip RMS가 커진다. 현재 구현은 최대 감소율을 0.25로 제한하여 두 KPI 사이의 균형을 맞췄다.

### 4.7 D1 DLC + brake 분석

D1에서는 sideSlipMax가 4.117 deg로 목표 4 deg보다 약간 커서 소폭 감점되었고, LTR_max는 0.744로 일부 감점되었다. lateralDevMax는 1.866 m로 목표 1 m를 넘어서 0점이 되었다.

D1은 A1의 double lane change와 B1의 제동이 결합된 시나리오이기 때문에, 현재 설계의 장점과 한계가 동시에 드러난다. ABS와 ESC는 차량이 크게 불안정해지는 것을 막지만, 보수적인 AFS와 yaw moment 제한 때문에 경로 추종 성능이 충분하지 않다. 또한 좌우 차동 제동은 yaw 안정성에는 도움이 되지만, 제동 중 하중이동으로 LTR을 증가시킬 수 있어 tuning trade-off가 발생한다.

---

## 5. 분석 및 한계

### 5.1 가장 성공적이었던 부분

가장 성공적인 부분은 A3, A7, B1이다. A3에서는 PD AFS 구조가 yaw rate step 응답을 빠르고 안정적으로 만들었고, A7에서는 ESC와 ABS가 결합되어 회전 중 급제동 안정성을 확보하였다. B1에서는 강한 감속 명령과 mild ABS modulation의 조합이 stopping distance와 slip RMS를 모두 만족시켰다.

특히 A7에서 sideSlipMax 2.950 deg, LTR_max 0.198을 기록한 것은 현재 통합 제어기의 안정성 중심 설계가 잘 작동한 결과이다. 급제동 상황에서 wheel slip을 억제하면서 yaw moment를 통해 차체 자세를 바로잡았기 때문이다.

### 5.2 부족했던 부분

가장 부족한 부분은 A1과 D1의 lateralDevMax이다. 두 시나리오 모두 lateralDevMax가 1.866 m로 나타났고, 목표값을 크게 초과하여 0점을 받았다. 이는 현재 제어기가 path tracking보다는 yaw rate와 sideslip 안정화에 초점을 맞추었기 때문이다.

또한 LTR_max도 A1과 D1에서 목표 0.6보다 큰 0.744 수준으로 나타났다. 이는 double lane change에서 빠른 좌우 하중이동이 발생하고, ESC yaw moment나 조향 보조가 하중이동을 충분히 줄이지 못했음을 의미한다. 수직 제어는 skyhook 방식으로 승차감 중심이며, roll/LTR 직접 저감 목표를 갖지는 않는다.

### 5.3 Gain tuning 과정에서 얻은 인식

초기에는 ESC yaw moment나 ABS 제동력을 더 강하게 하면 점수가 좋아질 것이라고 예상할 수 있다. 그러나 실제 KPI 기반 채점에서는 제어 입력이 강해질수록 다른 KPI가 악화되는 경우가 있었다. 예를 들어 lateral 안정성을 위해 yaw moment를 크게 주면 side slip은 줄 수 있지만 좌우 제동 비대칭 때문에 path deviation이나 LTR이 나빠질 수 있다. 반대로 AFS 조향 게인을 크게 하면 yaw rate 추종은 빨라지지만 overshoot와 lane change stability가 악화될 수 있다.

따라서 최종 설계는 공격적인 제어보다 보수적인 보조 제어를 택했다. 조향 보조를 ±2 deg로 제한하고, ESC threshold를 4~5 deg 수준으로 두며, ABS는 slip이 초과된 휠에 대해서만 완만하게 개입하도록 했다. 이 선택은 전체 정량점수 62.205/70으로 이어졌지만, path deviation 개선에는 한계가 있었다.

### 5.4 더 개선한다면

첫째, path tracking error를 직접 고려한 AFS 항을 추가할 수 있다. 현재 `ctrl_lateral`은 yaw rate error와 slip angle만 사용하므로 reference path와의 횡방향 오차를 직접 보지 못한다. 만약 runner에서 lateral deviation 또는 preview path 정보를 사용할 수 있다면, Stanley/Pure Pursuit 기반 보조항을 추가하여 A1/D1 lateralDevMax를 줄일 수 있다.

둘째, LTR 저감을 위해 vertical controller를 roll-aware CDC로 확장할 수 있다. 현재 skyhook은 각 휠의 수직속도만 사용한다. 좌우 감쇠 차이를 roll rate와 lateral acceleration에 따라 조절하면 double lane change에서 하중이동을 줄일 가능성이 있다.

셋째, actuator allocation을 weighted least squares 방식으로 바꿀 수 있다. 현재 coordinator는 rule-based 60:40 제동과 50:50 yaw moment 배분을 사용한다. WLS allocation을 적용하면 yaw moment 생성, 총 제동력 유지, 좌우 토크 변화 최소화, 휠별 토크 한계를 하나의 목적함수로 묶어 더 부드러운 통합 제어를 만들 수 있다.

---

## 6. 참고문헌

[1] ISO 3888-1:2018, *Passenger cars — Test track for a severe lane-change manoeuvre*.  
[2] ISO 4138:2021, *Passenger cars — Steady-state circular driving behaviour — Open-loop test methods*.  
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012.  
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008.  
[5] D. Hrovat, "Survey of advanced suspension developments and related optimal control applications," *Automatica*, vol. 33, no. 10, pp. 1781-1817, 1997.

---

## 부록 A — 사용한 AI 도구

`student_info.m`에는 다음과 같이 AI 도구 사용 내역을 작성하였다.

```matlab
info.ai_usage = 'Codex used for debugging, controller tuning assistance, and report drafting';
```

---

## 부록 B — 주요 코드 변경사항

### B.1 Lateral controller

초기 템플릿은 PID 구조였으나 최종 구현은 PD + derivative filtering + ESC limiter이다.

```matlab
gainScale = 1 + 0.15 * exp(-vx_safe / 10);
Kp = CTRL.LAT.Kp * gainScale * 0.7;
Ki = 0;
Kd = CTRL.LAT.Kd * 0.4;

yawDeadband = deg2rad(0.5);
slipDeadband = deg2rad(0.8);
steerCmd = max(-deg2rad(2.0), min(deg2rad(2.0), steerCmd));
```

ESC는 slip angle 기반으로 작동한다.

```matlab
beta_th = min(LIM.MAX_SLIP_ANGLE, deg2rad(4.0 + min(vx_safe / 20, 1.0) * 1.0));
Kbeta = 550;
yawMoment = -Kbeta * sign(slipAngle) * (abs(slipAngle) - beta_th) * speedFactor;
yawMoment = max(-1000, min(1000, yawMoment));
```

### B.2 Longitudinal controller

제동 상황에서는 PI 속도 추종 대신 최대 안전 감속과 ABS slip limiting을 사용한다.

```matlab
targetDecel = -min(0.95 * g, LIM.MAX_AX);
Fx_cmd = mass * targetDecel;

slipLimit = 0.12;
slipExcess = max(abs(prevSlipRatio) - slipLimit, 0);
forceCmd.brakeOffset = -scale * 0.6 * LIM.MAX_BRAKE_TRQ * brakeShare;
```

### B.3 Vertical controller

4륜 독립 on-off skyhook damping을 사용한다.

```matlab
if zs_dot(i) * (zs_dot(i) - zu_dot(i)) > 0
    dampingCmd(i) = CTRL.VER.cMax;
else
    dampingCmd(i) = CTRL.VER.cMin;
end
```

### B.4 Coordinator

종방향 제동은 60:40 전후 배분, ESC yaw moment는 좌우 차동 제동으로 변환한다.

```matlab
T_front = 0.6 * T_total;
T_rear  = 0.4 * T_total;

ratio_f = 0.5;
Mz_f = yawMoment * ratio_f;
Mz_r = yawMoment * (1 - ratio_f);
```
