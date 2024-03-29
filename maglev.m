clear all
close all
clc

raw_A = [0 1; 880.87 0];
B = [0; -9.9453];
C = [708.27 0];
D = 0;

% Uncomment the section with the desired reference behavior.

% Constant value R
R = 1;
poles = [-10 0];
initial_x = [0 -poles(1)/C(1)*R];

% Ramp of slope R
% R = 1;
% poles = [0 0];
% initial_x = [0 R/C(1)];

% Sine wave of amplitude a and frequency omega
% a = 1; omega = 1;
% poles = [+omega*i -omega*i];
% initial_x = [0 a/C(1)];

Obs_gain = place(raw_A', C', [-1 -2]).';
K_leader = acker(raw_A, B, poles);
A = raw_A - B*K_leader; % stabilized A (that will be used for distr. ctrl)

% Debug topology: all nodes are pinned
% pins = ones([1 6]);

% Chain topology: 1..6
% Gr = digraph(1:5, 2:6, [2 6 1 1 3]);
Gr = digraph(1:5, 2:6);
pins = [1 0 0 0 0 0];

% Tree topology (see report)
% Gr = digraph([1 1 4 4], [2 3 5 6], ones([4 1]));
% pins = [1 0 0 1 0 0];

% Fully connected topology
% Gr = digraph(ones([6 6]), 'omitselfloops');
% pins = [1 0 0 0 0 0];

% plot(Gr);
Adj = full(adjacency(Gr, 'weighted'));
adj_unw = full(adjacency(Gr));
% Adj = zeros(6); % no interconnections
in_degrees = sum(adj_unw, 1);
D_graph = diag(in_degrees);
L = D_graph - adj_unw;
G = diag(pins);
lambda_i = eig(L + G);

R = 5; Q = 5*eye(2);
P = are(A, B*inv(R)*(B'), Q);
K = inv(R)*(B')*P;

min_c = 1/(2*min(real(lambda_i)))
c = 0.6;
if c < min_c
    error("Invalid value for c")
end

% Select the desired observer type.
USE_LOCAL_OBSERVER = 1

if USE_LOCAL_OBSERVER
    F = place(A', -c*C', [-3 -4]).';
    if any(eig(A+c*F*C) > 0)
        error("A+cFC is not Hurwitz!")
    end
else
    R = 5; Q = eye(2);
    P = are(A', C'*inv(R)*C, Q);
    F = P*C'*inv(R);
end

% Set to 1 if you're not interested in running a parameter sweep
if 0
    c = 3*min_c;

    rise_times = zeros([6 3]);
    rise_low_thres = 0.1;
    rise_high_thres = 0.9;

    out = sim("maglev_sim");
    time = out.y_leader.Time;
    data = squeeze(out.y.Data);

    for j = 1:6
        rise_times(j, 1) = out.y.Time(find(data(j, :) > rise_low_thres, 1));
        rise_times(j, 2) = out.y.Time(find(data(j, :) > rise_high_thres, 1));
    end
    rise_times(:, 3) = rise_times(:, 2) - rise_times(:, 1);
        
    leader_rise_times = [0 0 0];
    leader_rise_times(1) = out.y_leader.Time(find(out.y_leader.Data > rise_low_thres, 1));
    leader_rise_times(2) = out.y_leader.Time(find(out.y_leader.Data > rise_high_thres, 1));
    leader_rise_times(3) = leader_rise_times(2) - leader_rise_times(1);

    leader_rise_times
    rise_times
else
    c_values = linspace(min_c + 0.1, 3*min_c, 20);
    
    all_rise_times = zeros([20 1]);
    
    for i = 1:20
        c = c_values(i)
        
        out = sim("maglev_sim");
        
        rise_times = zeros([6 2]);
        rise_low_thres = 0.1;
        rise_high_thres = 0.9;
        time = out.y_leader.Time;
        data = squeeze(out.y.Data);
    
        for j = 1:6
            rise_times(j, 1) = out.y.Time(find(data(j, :) > rise_low_thres, 1));
            rise_times(j, 2) = out.y.Time(find(data(j, :) > rise_high_thres, 1));
        end
            
        leader_rise_times = [0 0];
        leader_rise_times(1) = out.y_leader.Time(find(out.y_leader.Data > rise_low_thres, 1));
        leader_rise_times(2) = out.y_leader.Time(find(out.y_leader.Data > rise_high_thres, 1));
        
        all_rise_times(i) = rise_times(6, 2);
        rise_times(6, 2)
    end
    
    plot(c_values, all_rise_times)
end