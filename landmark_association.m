function indicator = landmark_association(xk, z1, z2, threshold)
indicator = 0;

x1 = xk(1) + z1(1) * cos(z1(2) + xk(3));
y1= xk(2) + z1(1) * sin(z1(2) + xk(3));

x2 = xk(1) + z2(1) * cos(z2(2) + xk(3));
y2= xk(2) + z2(1) * sin(z2(2) + xk(3));

if sqrt((x1-x2)^2 + (y1-y2)^2) < threshold
    indicator = 1;
end

end