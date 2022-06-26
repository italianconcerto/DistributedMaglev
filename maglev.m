A = [0 1; 880.87 0];
B = [0; -9.9453];
C = [708.27 0]; C = -C/100;
D = [0];

agent_sys = ss(A, B, C, D);
K_leader = place(A,B, [0 -20]);
Obs_gain = place(A', C', [-3 -5]);

% Debug topology: all nodes are pinned
% pins = ones([1 6]);

% Chain topology: 1..6
% Gr = digraph(1:5, 2:6, [2 6 1 1 3]);
% pins = [1 0 0 0 0 0];

% Tree topology (see report)
Gr = digraph([1 1 4 4], [2 3 5 6], ones([4 1]));
pins = [1 0 0 1 0 0];

% Fully connected topology
% Gr = digraph(ones([6 6]), 'omitselfloops');
% pins = [1 0 0 0 0 0];

plot(Gr);
Adj = full(adjacency(Gr, 'weighted'));
% Adj = zeros(6); % no interconnections
in_degrees = sum(Adj, 1);
D_graph = diag(in_degrees);
L = D_graph - Adj;
G = diag(pins);
lambda_i = eig(L + G);

R = 5; Q = 5*eye(2);
P = are(A, B*inv(R)*(B'), Q);
K = inv(R)*(B')*P;

min_c = 1/(2*min(real(lambda_i)))
c = 1.5*min_c;

P = are(A', C'*inv(R)*C, Q)
F = P*C'*inv(R);

out = sim('maglev_sim')

rise_times = zeros([6 2]);
rise_low_thres = 0.1;
rise_high_thres = 0.9;
rise_times(1, 1) = out.y1.Time(find(out.y1.Data > rise_low_thres, 1)) - 1;
rise_times(1, 2) = out.y1.Time(find(out.y1.Data > rise_high_thres, 1)) - 1;
rise_times(2, 1) = out.y1.Time(find(out.y2.Data > rise_low_thres, 1)) - 1;
rise_times(2, 2) = out.y1.Time(find(out.y2.Data > rise_high_thres, 1)) - 1;
rise_times(3, 1) = out.y1.Time(find(out.y3.Data > rise_low_thres, 1)) - 1;
rise_times(3, 2) = out.y1.Time(find(out.y3.Data > rise_high_thres, 1)) - 1;
rise_times(4, 1) = out.y1.Time(find(out.y4.Data > rise_low_thres, 1)) - 1;
rise_times(4, 2) = out.y1.Time(find(out.y4.Data > rise_high_thres, 1)) - 1;
rise_times(5, 1) = out.y1.Time(find(out.y5.Data > rise_low_thres, 1)) - 1;
rise_times(5, 2) = out.y1.Time(find(out.y5.Data > rise_high_thres, 1)) - 1;
rise_times(6, 1) = out.y1.Time(find(out.y6.Data > rise_low_thres, 1)) - 1;
rise_times(6, 2) = out.y1.Time(find(out.y6.Data > rise_high_thres, 1)) - 1;
rise_times

leader_rise_times = [0 0];
leader_rise_times(1) = out.y_leader.Time(find(out.y_leader.Data > rise_low_thres, 1)) - 1;
leader_rise_times(2) = out.y_leader.Time(find(out.y_leader.Data > rise_high_thres, 1)) - 1;
leader_rise_times

% plot(out.y1.Time, out.y1.Data)

