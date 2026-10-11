function [S_top, S_bot] = hb_sm_local_ctrl(sm_id, m_arm, i_arm, Vc_i, Vc_prev, Vc_next, t)
%#codegen
% ============ SUBMODULE LAYER : local voltage balancing + PSC-PWM ============
% Fully local: own capacitor voltage, the two ring neighbours, the arm modulation
% index, the arm current (sign only) and the time base.  No sorting and no access
% to the full Vc vector.  Executes at Ts (10 us).

N      = 8;         % SMs per arm  -> carrier phase shift (sm_id-1)/N
fsw    = 1250;      % carrier frequency per SM -> N*fsw = 10 kHz effective per arm
k_v    = 0.2;       % balancing gain (1/V)
dead   = 0.1;       % V, dead-band against chattering
Iscale = 1.0;       % A, smooth sign of the arm current

%% 1. neighbour-average consensus correction
err  = 0.5*(Vc_prev + Vc_next) - Vc_i;
if abs(err) < dead, err = 0; end
dirn = tanh(i_arm/Iscale);    % > 0 : inserting charges C  (both arms: i measured top -> bottom)
dm   = k_v*err*dirn;
m_i  = min(max(m_arm + dm, 0), 1);

%% 2. phase-shifted triangular carrier (0 -> 1 -> 0)
tau = mod(t*fsw + (sm_id - 1)/N, 1);
car = 1 - abs(2*tau - 1);

%% 3. half-bridge gating
if m_i >= car
    S_top = 1; S_bot = 0;     % insert  (+Vc)
else
    S_top = 0; S_bot = 1;     % bypass  (0 V)
end
end
