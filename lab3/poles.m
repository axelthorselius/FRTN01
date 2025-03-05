s = tf('s');

K = 2.613;
T = 0.452;

G = (2.25*K*(s*T+1)) / (s^2*T+0.12*s*T+2.25*s*K*T + 2.25*K);

A = [-0.12 0; 5 0];
B = [2.25; 0];
C = [0 1];

H = ss(A, B, C, 0);



p = [0.8+0.1i 0.8-0.1i]; % wanted poles

h = 0.05; % sampling ts

%H1 = ss(H.A, H.B, H.C, H.D, h); % needed??
H1 = c2d(H,0.05);

K_v = place(H1.A, H1.B, p)

kr = 1 / (H1.C * inv(eye(2) - H1.A + H1.B*K_v)*H1.B) %G(s=0)

Phi = H1.A
C = H1.C;
Gamma = H1.B
Phie = [Phi, Gamma ; zeros(1,2), 1];
Ce = [C 0];

p = [0.6+0.2i, 0.6-0.2i, 0.55];

Le = place(Phie', Ce', p)'
%Le = acker(Phie', Ce', p)'

Kbeta = K*0.5

khti = K*h/T