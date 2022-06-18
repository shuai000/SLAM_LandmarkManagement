function [temp_head, clusters] = candidate_landmark_association(temp_head, clusters, xk, association_threshold)

M = length(temp_head.association_counter);
N = length(clusters);

association_matrix = zeros(M, N);
distance_matrix = zeros(M, N);

for i=1:M
    for j=1:N
        [association_matrix(i,j), distance_matrix(i,j)] = landmark_association(xk, temp_head.centerinfo(i,:), clusters(j).ck, association_threshold);
    end
end

% check association matrix, each candidate landmark can only associate with
% one cluster, nearest neighbour rule
for j=1:N
    if sum(association_matrix(:,j))>1
        [~,index] = min(distance_matrix(:,j));
        association_matrix(:,j) = 0;
        association_matrix(index,j) = 1;
    end
end
 clusters_to_delete = [];
for i=1:M
    for j=1:N
        if association_matrix(i,j) == 1
            temp_head.association_counter(i) = temp_head.association_counter(i) + 1;
            
            temp_head.centerinfo(i, :) = clusters(j).ck;
            temp_head.detection_points(i) = size(clusters(j).z, 1);
            temp_head.xy(i, :) = mean(clusters(j).xy, 1);
            clusters_to_delete = [clusters_to_delete, j];
        end
    end
end
clusters(clusters_to_delete) = [];
end

function [indicator, dist] = landmark_association(xk, z1, z2, threshold)
indicator = 0;

x1 = xk(1) + z1(1) * cos(z1(2) + xk(3));
y1= xk(2) + z1(1) * sin(z1(2) + xk(3));

x2 = xk(1) + z2(1) * cos(z2(2) + xk(3));
y2= xk(2) + z2(1) * sin(z2(2) + xk(3));

dist = sqrt((x1-x2)^2 + (y1-y2)^2);
if dist < threshold
    indicator = 1;
end

end