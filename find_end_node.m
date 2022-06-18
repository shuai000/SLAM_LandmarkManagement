function [end_node, num] = find_end_node(head_node)
% num: the number of nodes in the list
num = 1;
temp = head_node;
while 1
    if isempty(temp.Next)
        break;
    end
    temp = temp.Next;
    num = num + 1;
end
end_node = temp;
end