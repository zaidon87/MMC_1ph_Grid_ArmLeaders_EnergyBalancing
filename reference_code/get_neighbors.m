function [Vc_i, Vc_prev, Vc_next] = get_neighbors(Vc_arm, i)
%#codegen
% Models the neighbour-to-neighbour link of the ring: returns the own capacitor
% voltage and the voltages of the previous / next (circular) neighbours.
% Only these three scalars reach the local controller.
N = length(Vc_arm);
k = min(max(round(i), 1), N);
Vc_i = Vc_arm(k);
if k == 1
    Vc_prev = Vc_arm(N);      % circular: SM1's previous neighbour is SMN
    Vc_next = Vc_arm(2);
elseif k == N
    Vc_prev = Vc_arm(N-1);
    Vc_next = Vc_arm(1);      % circular: SMN's next neighbour is SM1
else
    Vc_prev = Vc_arm(k-1);
    Vc_next = Vc_arm(k+1);
end
end
