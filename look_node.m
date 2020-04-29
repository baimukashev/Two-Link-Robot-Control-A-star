function [h, c] = look_node(list, node)
    h = 0;
    c = 0;
    for n=1:length(list)
        if node.x == list(n).x && node.y == list(n).y 
            h = 1;
            c = n;
            break
        end 
    end
end

% for n=1:length(closed_list)
%         if nbr.x == closed_list(n).x && nbr.y == closed_list(n).y 
%             nbr_in_closed = 1;
%             break
%         end 
%     end