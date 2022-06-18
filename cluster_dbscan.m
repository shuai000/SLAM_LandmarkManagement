function data_cluster = cluster_dbscan(z, xk, pk, threshold, R, Na, min_np)
% Cluster the measurements using the DBSCAN method
% xk: state vector
% z: vector for range, azimuth, SNR
% data_cluster, struct that has the following fields:
% z: measurements in polar coordinate for each cluster member
% xy: converted in xy coordinate, based on the current robot state
% ck: member in the cluster with the largest SNR
% dist: distance of ck w.r.t. each existing landmark
N = size(z, 1);
data = zeros(N, 2);
for i=1:N
    data(i, 1) = xk(1) + z(i, 1) * cos(z(i, 2) + xk(3));
    data(i, 2) = xk(2) + z(i, 1) * sin(z(i, 2) + xk(3));
end

idx = dbscan(data(:,1:2), threshold, min_np);

for i=1:max(idx)
    index_i = find(idx==i);
    num_i = length(index_i);
    data_cluster(i).z = zeros(num_i, 3);
    data_cluster(i).xy = zeros(num_i, 2);
    for ic=1:num_i
        data_cluster(i).z(ic,:) = z(index_i(ic),:);
        data_cluster(i).xy(ic,:) = data(index_i(ic),:);
    end
end

if max(idx)==-1
    data_cluster = [];
end

num_landmarks = (length(xk) - 3)/2;
landmarks = reshape(xk(4:end), 2, num_landmarks)';

for i=1:length(data_cluster)
    [~, max_index] = max(data_cluster(i).z(:, 3)); % the highest SNR
    data_cluster(i).ck = data_cluster(i).z(max_index, :);
    if num_landmarks > 0
        % sort Eudician D and compute the log D for the first Nc landmarks
        dist_vector = sqrt(sum((data_cluster(i).xy(max_index, 1:2) - landmarks).^2, 2));
        [~, sorted_index] = sort(dist_vector);
        Nc = min(Na, length(sorted_index));        
        % select the first Nc landmarks to compute log distance (computation
        % issue)
        logD = [];
        for iu=1:Nc
            j = sorted_index(iu); % the jth landmark
            logD(iu) = log_distance(data_cluster(i).ck(1:2), xk, pk, landmarks(j, :), R, j);
        end
        data_cluster(i).logD = logD;
    else
        data_cluster(i).logD = Inf; % currently no landmarks included in the state
    end
end

end

function min_dis = minimun_dist(data, cluster_points)

for i=1:size(cluster_points, 1)
    distance(i) = sqrt( (data(1) - cluster_points(i, 1))^2 + (data(2) - cluster_points(i, 2))^2 );
end
min_dis = min(distance);
end