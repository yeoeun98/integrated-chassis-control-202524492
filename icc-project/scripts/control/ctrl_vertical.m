function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

    %% Skyhook 기반 CDC 설계
    if ~isfield(ctrlState, 'prevDamping')
        ctrlState.prevDamping = CTRL.VER.cMin * ones(4,1);
    end

    zs_dot = reshape(suspState.zs_dot, [], 1);
    zu_dot = reshape(suspState.zu_dot, [], 1);
    dampingCmd = CTRL.VER.cMin * ones(numel(zs_dot), 1);
    for i = 1:numel(zs_dot)
        if zs_dot(i) * (zs_dot(i) - zu_dot(i)) > 0
            dampingCmd(i) = CTRL.VER.cMax;
        else
            dampingCmd(i) = CTRL.VER.cMin;
        end
    end
    dampingCmd = min(max(dampingCmd, CTRL.VER.cMin), CTRL.VER.cMax);

    ctrlState.prevDamping = dampingCmd;

end
