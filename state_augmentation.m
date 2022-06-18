function [xa, pa, Lx, Ly] = state_augmentation(xk, pk, z, R)
% state augmentation function
% see Eq.(16) in the Appendix of the paper
N = length(xk);

Lx = xk(1) + z(1) * cos(z(2) + xk(3));
Ly = xk(2) + z(1) * sin(z(2) + xk(3));

% state augamentation
xa = [xk; Lx; Ly];

psi = z(2) + xk(3);
rk = z(1);

J1 = zeros(N+2, N);
for i=1:N
    J1(i,i) = 1;
end
J1(N+1, 1) = 1;
J1(N+1, 3) = -rk*sin(psi);
J1(N+2, 2) = 1;
J1(N+2, 3) = rk*cos(psi);

J2 = zeros(N+2, 2);
J2(N+1, 1) = cos(psi);
J2(N+1, 2) = -rk * sin(psi);
J2(N+2, 1) = sin(psi);
J2(N+2, 2) = rk * cos(psi);

pa = J1 * pk * J1' + J2 * R * J2';

end