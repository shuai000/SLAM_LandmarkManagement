function deleting_index = find_to_delete(temp_head, M, N)
deleting_index = [];
for i = 1:length(temp_head.association_counter)
    if temp_head.window_length(i) >=M || temp_head.association_counter(i) + M - temp_head.window_length(i) < N
        deleting_index = [deleting_index, i];
    end
end
end