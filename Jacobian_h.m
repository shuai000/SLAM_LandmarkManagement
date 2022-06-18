function H = Jacobian_h(xk, j)
% compute the numerical Jacobian matrix for measurement model

N = length(xk);
H = zeros(2, N);

j1 = 3 + (j-1)*2 + 1;
j2 = 3 + (j-1)*2 + 2;

Lx = xk(j1);
Ly = xk(j2);

x = xk(1);      
y = xk(2);

rk = sqrt( (Lx - x)^2 + (Ly - y)^2 );
dx = Lx - x;
dy = Ly - y;

H(1,1) = - dx/rk; H(1,2) = - dy/rk; H(1,j1) = dx/rk; H(1,j2) = dy/rk;

H(2,1) = dy/(rk^2); H(2,2) = - dx/(rk^2); H(2,3) = -1;  % IMPORTANT DOUBLE CHECK THIS ITEM
H(2,j1) = -dy/(rk^2); H(2,j2) = dx/(rk^2);

end