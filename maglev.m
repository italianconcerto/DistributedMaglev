clear all;
close all;
clc;

A = [0 1;
    880.87 0
];

B = [0; -9.9453];

C = [7.0827 0];

D = [0];

agent_sys = ss(A, B, C, D)
K_leader = place(A,B, [-0.000001 -0.000092]);

Gr = digraph(1:5, 2:6, [2 6 1 1 3]);
Adj = full(adjacency(Gr))
in_degrees = sum(Adj, 1);
D_graph = diag(in_degrees);
L = D_graph - Adj;
pins = [1 0 0 0 0 0];
G = diag(pins);
lambda_i = eig(L + G);

R = 5; Q = 5*eye(2);
P = are(A, B*inv(R)*(B'), Q);

K = inv(R)*(B')*P

min_c = 1/(2*min(real(lambda_i)))
c = 0.7;
