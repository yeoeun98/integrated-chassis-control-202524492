function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% 1) steer allocation
    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    %% 2) longitudinal brake base allocation
    T_total = 0;
    if isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        T_total = min(4 * LIM.MAX_BRAKE_TRQ, max(0, -lonCmd.Fx_total * VEH.rw / 4));
    end
    if T_total == 0 && isfield(lonCmd, 'brakeRatio')
        T_total = min(4 * LIM.MAX_BRAKE_TRQ, max(0, lonCmd.brakeRatio * 4 * LIM.MAX_BRAKE_TRQ));
    end
    T_front = 0.6 * T_total;
    T_rear  = 0.4 * T_total;
    brakeFL = T_front / 2;
    brakeFR = T_front / 2;
    brakeRL = T_rear  / 2;
    brakeRR = T_rear  / 2;

    %% 3) ESC yaw moment allocation
    if isfield(latCmd, 'yawMoment') && abs(latCmd.yawMoment) > 0
        ratio_f = 0.5;
        yawMoment = max(-1500, min(1500, latCmd.yawMoment));
        % 기존 differential 배분이 음수 휠 토크를 만들면 전체 브레이크가 급격히 감소하므로,
        % 가능한 yaw moment를 현재 longitudinal base 브레이크 한도 안으로 제한한다.
        maxMz_f = 2 * brakeFR * (VEH.track_f / 2);
        minMz_f = -2 * brakeFL * (VEH.track_f / 2);
        maxMz_r = 2 * brakeRR * (VEH.track_r / 2);
        minMz_r = -2 * brakeRL * (VEH.track_r / 2);
        if yawMoment >= 0
            yawMoment = min(yawMoment, min(maxMz_f/ratio_f, maxMz_r/(1-ratio_f)));
        else
            yawMoment = max(yawMoment, max(minMz_f/ratio_f, minMz_r/(1-ratio_f)));
        end
        Mz_f = yawMoment * ratio_f;
        Mz_r = yawMoment * (1 - ratio_f);
        delta_f = Mz_f / max(VEH.track_f / 2, eps);
        delta_r = Mz_r / max(VEH.track_r / 2, eps);
        brakeFL = brakeFL + delta_f / 2;
        brakeFR = brakeFR - delta_f / 2;
        brakeRL = brakeRL + delta_r / 2;
        brakeRR = brakeRR - delta_r / 2;
    end

    brakeTorque = [brakeFL; brakeFR; brakeRL; brakeRR];
    brakeTorque = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, brakeTorque));

    actuatorCmd.brakeTorque = brakeTorque;
    actuatorCmd.dampingCoeff = min(max(verCmd(:), CTRL.VER.cMin), CTRL.VER.cMax);

end
