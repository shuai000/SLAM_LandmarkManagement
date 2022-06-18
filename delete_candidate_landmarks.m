function temp_head = delete_candidate_landmarks(temp_head, deleting_index)

temp_head.association_counter(deleting_index) = [];
temp_head.centerinfo(deleting_index, :)  = [];
temp_head.window_length(deleting_index) = [];
temp_head.detection_points(deleting_index) = [];
temp_head.xy(deleting_index, :) = [];

end