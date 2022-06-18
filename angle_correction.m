function angle_output = angle_correction(angle_input)
% address the 2pi rapping for robot angle

angle_output = angle_input;

if angle_input >= 2*pi
    angle_output = angle_input - 2*pi;
end

if angle_input < 0
    angle_output = angle_input + 2*pi;
end

end