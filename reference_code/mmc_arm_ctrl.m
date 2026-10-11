function [m_arm, W_arm, dbg] = mmc_arm_ctrl(i_ac_ref, theta, V_amp, P_meas, v_ac, i_arm, Vc_arm, W_other, IsUpper, Vdc, Vc_nom)
%#codegen
% ========== INNER LOOP + INTER-ARM ENERGY BALANCING  (one instance per arm) ==========
% Executes at Ts_ctrl.  The only information this arm needs from the OTHER arm is
% one scalar: its raw stored energy W_other (received through a one-period unit
% delay that models the communication latency).  With s = +1 (upper) / -1 (lower)
% both leaders compute the SAME i_circ_ref, so no further coordination is needed.
%
%   m_arm : modulation index of this arm (0..1)      -> SM layer (local PSC-PWM)
%   W_arm : raw energy of this arm (J)               -> sent to the other arm
%   dbg   : [i_ref; i_circ_ref; v_arm_ref; W_own_f; W_other_f]
%   Vdc, Vc_nom : DC-link voltage and nominal SM voltage, read from the workspace (VDC, vo)
%                 through Constant blocks.  N*Vc_nom may exceed Vdc (stack margin for |v_ac| -> Vdc/2).

Ts     = 1e-4;
C      = 2.2e-3;
N      = numel(Vc_arm);
w0     = 2*pi*50;
NMA    = 200;                 % 20 ms moving average on the energies (nulls 50/100 Hz)

% --- arm current loop: PI + resonant terms at 50 Hz and 100 Hz (plant L = 5 mH, R = 0.5 ohm) -> f_bw ~ 380 Hz
Kp_i = 12;    Ki_i = 1200;    Kr_i = 300;   xi_i_max = 300;   xr_max = 200;   v_pi_max = 400;
% --- CCSC: resonant term at 2*w0 removes the 100 Hz circulating current from THIS arm's current.
%     i_ref carries no 2w0 component, so any 2w0 content of i_arm is error -> driven to zero locally,
%     without knowing the other arm's current (single-phase equivalent of the dq -2w CCSC).
USE_CCSC = true;   Kr_2 = 300;   xr2_max = 150;
% --- total-energy loop  (dW_sum/dt = Vdc*i_dc - P_ac)          -> f_bw ~ 4 Hz
Kp_s = 0.03;  Ki_s = 0.3;     xi_s_max = 3;     i_dc_max  = 5;
% --- differential-energy loop (dW_dif/dt = -V_amp*I_dif)       -> wn ~ 12 rad/s, critically damped
Kp_d = 25;    Ki_d = 150;     xi_d_max = 300;   I_dif_max = 3;

persistent xi_i xr1 xr2 xc1 xc2 xi_s xi_d bufO bufX idx
if isempty(xi_i)
    xi_i = 0; xr1 = 0; xr2 = 0; xc1 = 0; xc2 = 0; xi_s = 0; xi_d = 0;
    W0   = 0.5*C*N*Vc_nom^2;
    bufO = W0*ones(NMA,1); bufX = W0*ones(NMA,1); idx = 1;
end
if IsUpper > 0.5, s = 1; else, s = -1; end

%% 1. own arm energy (raw) -> exported to the other arm
W_arm = 0.5*C*sum(Vc_arm.^2);
W_nom = 0.5*C*N*Vc_nom^2;

%% 2. energy filtering (both arms use identical filters -> identical results)
bufO(idx) = W_arm;
bufX(idx) = W_other;
idx = idx + 1; if idx > NMA, idx = 1; end
Wo = sum(bufO)/NMA;           % own   (filtered)
Wx = sum(bufX)/NMA;           % other (filtered)

%% 3. energy-balancing layer
% 3a. total energy -> DC circulating current (identical in both arms)
e_sum   = 2*W_nom - (Wo + Wx);
i_dc_un = P_meas/Vdc + Kp_s*e_sum + xi_s;                        % unsaturated
if abs(i_dc_un) < i_dc_max                                       % clamping anti-windup
    xi_s = min(max(xi_s + Ki_s*Ts*e_sum, -xi_s_max), xi_s_max);
end
i_dc_ref = min(max(i_dc_un, -i_dc_max), i_dc_max);

% 3b. differential energy (own - other) -> fundamental circulating current in phase
%     with v_ac.  s flips the sign so that both arms demand the SAME I_dif:
%     upper: +K*(Wu-Wl)   lower: -K*(Wl-Wu) = +K*(Wu-Wl)
e_dif    = Wo - Wx;
u_dif    = Kp_d*e_dif + xi_d;                                    % W
I_dif_un = s*u_dif/max(V_amp, 50);                               % A (unsaturated)
if abs(I_dif_un) < I_dif_max                                     % clamping anti-windup
    xi_d = min(max(xi_d + Ki_d*Ts*e_dif, -xi_d_max), xi_d_max);
end
I_dif = min(max(I_dif_un, -I_dif_max), I_dif_max);
i_circ_ref = i_dc_ref + I_dif*cos(theta);

%% 4. arm current reference and inner PI + R
%     i_u = +i_ac/2 + i_circ ,  i_l = -i_ac/2 + i_circ   (both measured top -> bottom)
i_ref = s*0.5*i_ac_ref + i_circ_ref;
err_i = i_ref - i_arm;
xi_i  = min(max(xi_i + Ki_i*Ts*err_i, -xi_i_max), xi_i_max);
% resonant integrator at w0 (symplectic Euler: |lambda| = 1, no drift)
xr1   = xr1 + Ts*(Kr_i*err_i - w0*xr2);
xr2   = xr2 + Ts*w0*xr1;
xr1   = min(max(xr1, -xr_max), xr_max);
% resonant integrator at 2*w0 (CCSC)
if USE_CCSC
    xc1 = xc1 + Ts*(Kr_2*err_i - 2*w0*xc2);
    xc2 = xc2 + Ts*2*w0*xc1;
    xc1 = min(max(xc1, -xr2_max), xr2_max);
else
    xc1 = 0; xc2 = 0;
end
v_pi  = min(max(Kp_i*err_i + xi_i + xr1 + xc1, -v_pi_max), v_pi_max);

%% 5. arm voltage reference -> modulation index
%     upper: v_u = Vdc/2 - v_ac - v_pi      lower: v_l = Vdc/2 + v_ac - v_pi
v_arm_ref = Vdc/2 - s*v_ac - v_pi;
V_stack   = max(sum(Vc_arm), 0.5*N*Vc_nom);        % measured stack voltage
m_arm     = min(max(v_arm_ref/V_stack, 0), 1);

dbg = [i_ref; i_circ_ref; v_arm_ref; Wo; Wx];
end
