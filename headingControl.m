function heading_now = headingControl(heading_previous, input, p_noise)
% Compute the robot heading after applying a control input (can be + or -)
% Robot heading is defined w.r.t horizontal, pointing to the right, ...>
% Range of the heading is defined [0, 2*pi) in radian
% This is part of the robot motion model
if nargin==2
    p_noise = 0;
end
heading_now = heading_previous + input + p_noise;

if heading_now < 0
    heading_now = heading_now + 2*pi;
end

if heading_now >= 2*pi
    heading_now = heading_now - 2*pi;
end

end

