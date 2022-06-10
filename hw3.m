m1 = 1.1; m2 = 0.9; k1 = 1.5; k2 = 1;
A = [
    0 1 0 0;               % dx1/dt = x2 (x1 = displacement of m1, x2 = velocity of m1)
    (-k1-k2)/m1 0 k2/m1 0; % force of m1 = -(k1+k2) * displacement of m1 + k2 * displacement of m2
    0 0 0 1;               % dx3/dt = x4 (x3 = displacement of m2, x4 = velocity of m2)
    k2/m2 0 -k2/m2 0
];
B = [0; 1/m1; 0; 0];
C = eye(4);
D = zeros([4 1]);
agent_sys = ss(A, B, C, D)

Gr = digraph(1:5, 2:6, [2 6 1 1 3]);
Adj = full(adjacency(Gr));
in_degrees = sum(Adj, 1);
D = diag(in_degrees);
L = D - Adj;
pins = [1 0 0 0 0 0];
G = diag(pins);
lambda_i = eig(L + G);

R = 5; Q = 5*eye(4);
P = are(A, B*inv(R)*(B'), Q);
K = inv(R)*(B')*P
min_c = 1/(2*min(real(lambda_i)))
c = 0.7;