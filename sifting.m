function [sk, index] = sifting(zk, xk, ridus_threshold)
% Select measurement candidate for state update
% output:
% sk: selected measurement
% index: corresponding index in zk
num_landmarks = (length(xk) - 3)/2;
if num_landmarks == 0
    sk = [];
    index = [];
else
    landmarks = reshape(xk(4:end), 2, num_landmarks)';
    sk = zeros(1, 3);
    N = size(zk, 1);
    landmark_xy = zeros(N, 2);
    for i=1:N
        landmark_xy(i, 1) = xk(1) + zk(i, 1) * cos(zk(i, 2) + xk(3));
        landmark_xy(i, 2) = xk(2) + zk(i, 1) * sin(zk(i, 2) + xk(3));
    end
    counter = 1;
    index = [];
    for i=1:num_landmarks
        difference_value = landmarks(i, :) - landmark_xy;
        distance_i = sqrt(sum(difference_value.^2, 2));
        
        for j=1:length(distance_i)
            if distance_i(j) < ridus_threshold && ~ismember(j, index)
                sk(counter, :) = zk(j, :);
                index = [index, j];
                counter = counter + 1;
            end
        end
    end
end

end