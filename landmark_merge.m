function [state, id] = landmark_merge(state, t, id_list, threshold_xy, threshold_p)
% to merge two landmarks if they are very close to each other
% Refer to Lanmark Merging in the paper

Lk_position =  reshape(state(t).x_update(6:end), [2, length(state(t).x_update(6:end))/2])';
N = size(Lk_position, 1);

x_dist = zeros(N,N);
y_dist = zeros(N,N);
p_dist = zeros(N,N);
id = [];

for i=1:N
    for j=1:N
        x_dist(i,j) = abs(Lk_position(i,1) - Lk_position(j,1));
        y_dist(i,j) = abs(Lk_position(i,2) - Lk_position(j,2));
        p_dist(i,j) = sqrt(x_dist(i,j)^2 + y_dist(i,j)^2);
    end
end
merge_indicator = zeros(N,N);
for i=1:N-1
    for j=i+1:N
        if x_dist(i,j) <= threshold_xy || y_dist(i,j) <= threshold_xy
            if p_dist(i,j) <= threshold_p
                % eligible to be merged
                merge_indicator(i,j) = 1;
            end
        end
    end
end

% Action
for i=1:N-1
    if sum(merge_indicator(i,:)) > 0
        index = find(merge_indicator(i,:) == 1);
        if length(index)>1
            [~, min_index] = min(dist_p(i, :));
            merge_indicator(i,:) = 0;
            merge_indicator(i,min_index) = 1;
        end
        % merge
        remove_index = find(merge_indicator(i,:) == 1);
        from_index = 6+2*(remove_index-1);
        state(t).x_update(from_index:from_index+1) = [];
        state(t).cov_update([from_index, from_index+1], :) = [];
        state(t).cov_update(:, [from_index, from_index+1]) = [];
        id = [id, id_list(remove_index)];
        break;
    end
end

end