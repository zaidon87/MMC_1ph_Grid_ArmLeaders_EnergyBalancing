function [i_ac_ref, theta, V_amp, P_meas, Q_meas] = mmc_outer_ctrl(v_ac, i_ac, Pstar, Qstar, t)
%#codegen
% ================= OUTER LOOP - GRID-CONNECTED P/Q CONTROL (one instance per arm) =================
% Executes at Ts_ctrl (Rate Transition blocks on every input pin the rate).
% Both arm leaders run this same deterministic code on the same measurements, so
% they produce an identical i_ac_ref without exchanging anything.
%
%   SOGI on v_ac and i_ac -> orthogonal (alpha, beta) components
%   SOGI-PLL (normalised)  -> theta locked to the PCC voltage (v_alpha = V_amp*cos(theta))
%   P = 0.5*(va*ia + vb*ib) ,  Q = 0.5*(va*ib - vb*ia)   [Q > 0 : current LEADS voltage, capacitive]
%   i_ac_ref = Ip*cos(theta) - Iq*sin(theta)
%
%   USE_PLL = false falls back to an internal 50 Hz oscillator (standalone resistive load).

Ts      = 1e-4;              % execution period (s) == Ts_ctrl in the workspace
f0      = 50;  w0 = 2*pi*f0;
USE_PLL = true;              % true : grid-connected (PLL on v_ac)   false : internal oscillator
NMA     = 200;               % 20 ms moving average on P and Q (nulls 50/100 Hz residuals)
T_START = 0.15;              % s, references held at zero until the PLL has locked
PQ_RAMP = 10000;             % W/s (var/s), reference ramp after T_START (soft start)

% --- SOGI and PLL
k_sogi  = 1.41;              % SOGI gain -> bandwidth ~ k*f0/2 = 35 Hz
Kp_pll  = 50;  Ki_pll = 1000;   % on vq/V_amp (p.u.) -> wn = 32 rad/s (5 Hz), zeta = 0.79
dw_max  = 2*pi*5;            % rad/s, PLL integrator limit (+-5 Hz)

% --- P and Q loops (plant: P = 0.5*V_amp*Ip, ~163 W/A at 325 V) -> f_bw ~ 5 Hz
Kp_pq   = 0.004;             % A/W   (loop gain 0.65 with the 10 ms MA delay)
Ki_pq   = 0.2;               % A/(W*s)
I_max   = 15;                % A, amplitude limit of each current component
P_max   = 3000;              % W
Q_max   = 2000;              % var
tau_v   = 0.01;              % s, LPF on the SOGI magnitude

persistent th pll_i vs1 vs2 is1 is2 Vf bufP bufQ idx xi_p xi_q
if isempty(th)
    th = 0; pll_i = 0; vs1 = 0; vs2 = 0; is1 = 0; is2 = 0; Vf = 300;
    bufP = zeros(NMA,1); bufQ = zeros(NMA,1); idx = 1;
    xi_p = 0; xi_q = 0;
end

%% 1. SOGI (voltage and current), symplectic Euler at 10 kHz (unity gain at w0; forward Euler gives +2.3 %)
vs1 = vs1 + Ts*(k_sogi*w0*(v_ac - vs1) - w0*vs2);
vs2 = vs2 + Ts*w0*vs1;
is1 = is1 + Ts*(k_sogi*w0*(i_ac - is1) - w0*is2);
is2 = is2 + Ts*w0*is1;
va = vs1; vb = vs2; ia = is1; ib = is2;

%% 2. amplitude and angle
Vn = max(sqrt(va^2 + vb^2), 20);
Vf = Vf + (Ts/(tau_v + Ts))*(Vn - Vf);
V_amp = Vf;
if USE_PLL
    vq    = (-sin(th)*va + cos(th)*vb)/Vn;            % normalised q-axis error (p.u.)
    pll_i = min(max(pll_i + Ki_pll*Ts*vq, -dw_max), dw_max);
    th    = mod(th + (w0 + Kp_pll*vq + pll_i)*Ts, 2*pi);
else
    th    = mod(w0*t, 2*pi);
end
theta = th;

%% 3. measured P and Q (instantaneous from alpha/beta, then 20 ms average)
bufP(idx) = 0.5*(va*ia + vb*ib);
bufQ(idx) = 0.5*(va*ib - vb*ia);
idx = idx + 1; if idx > NMA, idx = 1; end
P_meas = sum(bufP)/NMA;
Q_meas = sum(bufQ)/NMA;

%% 4. P and Q loops -> current components (feed-forward + PI, clamping anti-windup)
if t < T_START
    P_use = 0; Q_use = 0;                             % wait for PLL lock
else
    lim   = PQ_RAMP*(t - T_START);                    % ramped envelope
    P_use = min(max(min(max(Pstar, -P_max), P_max), -lim), lim);
    Q_use = min(max(min(max(Qstar, -Q_max), Q_max), -lim), lim);
end
Ip_ff = 2*P_use/V_amp;   Iq_ff = 2*Q_use/V_amp;
e_p = P_use - P_meas;    e_q = Q_use - Q_meas;
Ip_un = Ip_ff + Kp_pq*e_p + xi_p;
if abs(Ip_un) < I_max, xi_p = min(max(xi_p + Ki_pq*Ts*e_p, -I_max), I_max); end
Iq_un = Iq_ff + Kp_pq*e_q + xi_q;
if abs(Iq_un) < I_max, xi_q = min(max(xi_q + Ki_pq*Ts*e_q, -I_max), I_max); end
Ip = min(max(Ip_un, -I_max), I_max);
Iq = min(max(Iq_un, -I_max), I_max);

%% 5. AC current reference
i_ac_ref = Ip*cos(theta) - Iq*sin(theta);
end
