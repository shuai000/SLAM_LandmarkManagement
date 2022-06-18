function landmark_index = sift_association(sk, xk, pk, R, beta, Ns)
% Apply a finer association based on the sifting results and 
% a log likelihood based distance metric

num_landmarks = (length(xk) - 3)/2;
landmarks = reshape(xk(4:end), 2, num_landmarks)';

sk_x = xk(1) + sk(1) * cos(sk(2) + xk(3));
sk_y = xk(2) + sk(1) * sin(sk(2) + xk(3));

difference_value = [sk_x, sk_y] - landmarks;
distance_i = sqrt(sum(difference_value.^2, 2));

[~, sorted_index] = sort(distance_i);
Ns = min(Ns, length(distance_i));
% select the first Ns landmarks as candidates to be associated with sk
for i=1:Ns
    j = sorted_index(i); % the jth landmark
    logD(i) = log_distance(sk(1:2), xk, pk, landmarks(j, :), R, j);
end

[min_logD, min_index] = min(logD);

if min_logD < beta
    landmark_index = sorted_index(min_index);
else
    landmark_index = -1; % not associated
end

end