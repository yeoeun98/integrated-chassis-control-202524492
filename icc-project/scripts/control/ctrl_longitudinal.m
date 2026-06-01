function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, brakeActive, prevSlipRatio, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       brakeActive - 시나리오가 브레이크를 강제하는 중인지 여부 (true/false)
%       prevSlipRatio - 이전 스텝 휠별 slip ratio [4x1]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% Controller state initialize
    if ~isfield(ctrlState, 'intError'); ctrlState.intError = 0; end
    if ~isfield(ctrlState, 'prevForce'); ctrlState.prevForce = 0; end

    %% 1) speed tracking PI
    speedErr = vxRef - vx;
    if brakeActive
        % 브레이크 중에는 속도 유지 명령으로 제어기의 가속 개입을 막는다.
        speedErr = 0;
        ctrlState.intError = 0;
    else
        ctrlState.intError = ctrlState.intError + speedErr * dt;
        ctrlState.intError = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.intError));
    end

        if brakeActive
            % 브레이크 상황에서 open-loop 제동력을 넘는 전체 감속 목표를 설정
            mass = 1500;
            g = 9.81;
            targetDecel = -min(0.9 * g, LIM.MAX_AX);   % 최대 안전 제동가속도
            Fx_cmd = mass * targetDecel;
            if vx < 1.0
                Fx_cmd = 0;
            end
            % 이미 충분히 감속 중이면 제동 토크를 조금 줄여 안정화
            if ax < 1.2 * targetDecel
                Fx_cmd = Fx_cmd * 0.8;
            end
        else
            Fx_cmd = CTRL.LON.Kp * speedErr + CTRL.LON.Ki * ctrlState.intError;
            if Fx_cmd < 0 && ax < -0.5
                brakeReduction = 0.75 + 0.15 * exp(-abs(ax) / 3);
                Fx_cmd = Fx_cmd * brakeReduction;
            end
        end

    %% 3) jerk limit (approximate mass 1500 kg)
    mass = 1500;
    maxDeltaF = 3 * LIM.MAX_JERK * mass * dt;   % 더 빠른 brake torque ramp-up 허용
    Fx_cmd = max(ctrlState.prevForce - maxDeltaF, min(ctrlState.prevForce + maxDeltaF, Fx_cmd));

    %% 4) wheel-slip 기반 ABS offset
    forceCmd.brakeOffset = zeros(4,1);
    if brakeActive && nargin >= 6 && ~isempty(prevSlipRatio)
        slipLimit = 0.12;
        slipExcess = max(abs(prevSlipRatio) - slipLimit, 0);
        if any(slipExcess > 0) && ax < -0.2
            % 휠별 slip이 과도할 때만 해당 휠의 제동 토크를 줄인다.
            scale = min(0.20, max(0.05, max(slipExcess) / 0.30));
            brakeShare = slipExcess ./ max(slipExcess, eps);
            forceCmd.brakeOffset = -scale * LIM.MAX_BRAKE_TRQ * brakeShare;
        end
    end

    %% 5) output normalisation
    maxForce = mass * LIM.MAX_AX;
    Fx_cmd = max(-maxForce, min(maxForce, Fx_cmd));
    if Fx_cmd < -0.9 * maxForce
        Fx_cmd = -0.9 * maxForce;
    end
    if Fx_cmd < 0
        forceCmd.brakeRatio = min(1, max(0, -Fx_cmd / maxForce));
    else
        forceCmd.brakeRatio = 0;
    end

    forceCmd.Fx_total = Fx_cmd;
    ctrlState.prevForce = Fx_cmd;

end
