function headData = add_node_to_landmark(lx, ly, time, counter_landmark)
% Create the head in the link list for each landmark
headData.ID = counter_landmark;
headData.position = [lx, ly];
headData.createTime = time;
headData.removeTime = -1;
headData.association = 1;
headData.withinRange = 1;

end