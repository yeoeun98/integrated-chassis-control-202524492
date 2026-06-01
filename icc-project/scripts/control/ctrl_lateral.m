function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% Controller state initialize
    if ~isfield(ctrlState, 'intError'); ctrlState.intError = 0; end
    if ~isfield(ctrlState, 'prevError'); ctrlState.prevError = 0; end

    %% 1) yaw rate error and speed scheduling
    yawErr = yawRateRef - yawRate;
    vx_safe = max(vx, 0.1);
    gainScale = 1 + 0.2 * exp(-vx_safe / 12);   % 저속에서 더 강하지만 고속에선 안정화
    Kp = CTRL.LAT.Kp * gainScale * 1.2;
    Ki = 0;                                     % 경로 추종은 driver가 주도하므로 적분 제거
    Kd = CTRL.LAT.Kd * 0.5;                    % yaw disturbance damping

    %% 2) PID control with anti-windup
    ctrlState.intError = ctrlState.intError + yawErr * dt;
    ctrlState.intError = max(-CTRL.LAT.intMax, min(CTRL.LAT.intMax, ctrlState.intError));
    dError = (yawErr - ctrlState.prevError) / max(dt, eps);
    steerCmd = Kp * yawErr + Kd * dError;
    steerCmd = max(-deg2rad(1.2), min(deg2rad(1.2), steerCmd));

    %% 3) slip angle 기반 ESC yaw moment
    beta_th = min(LIM.MAX_SLIP_ANGLE, deg2rad(3.0 + min(vx_safe / 20, 1.0) * 1.5));
    yawMoment = 0;
    if abs(slipAngle) > beta_th
        Kbeta = 450;
        speedFactor = min(max(vx_safe / 15, 0.2), 1.0);
        yawMoment = -Kbeta * sign(slipAngle) * (abs(slipAngle) - beta_th) * speedFactor;
    end
    yawMoment = max(-1000, min(1000, yawMoment));

    %% 4) output saturation
    deltaAdd.steerAngle = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steerCmd));
    deltaAdd.yawMoment  = yawMoment;

    %% update state
    ctrlState.prevError = yawErr;

end
