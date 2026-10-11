function [v_ac_f, i_ac_f, i_u_f, i_l_f] = meas_avg(v_ac, i_ac, i_u, i_l)
%#codegen
% Measurement conditioning for the 10 kHz control layer.
% 10-sample boxcar at Ts = 10 us  ->  exact nulls at 10 kHz (= N*fsw, the dominant
% PWM ripple of each arm) and at its multiples, so the Rate Transition blocks that
% follow do not alias the switching ripple into the control loops.
M = 10;
persistent buf idx
if isempty(buf), buf = zeros(M,4); idx = 1; end
buf(idx,:) = [v_ac, i_ac, i_u, i_l];
idx = idx + 1; if idx > M, idx = 1; end
y = sum(buf,1)/M;
v_ac_f = y(1);
i_ac_f = y(2);
i_u_f  = y(3);
i_l_f  = y(4);
end
