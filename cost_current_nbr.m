function out = cost_current_nbr(current_node, nbr)
    if abs(nbr.x-current_node.x) + abs(nbr.y-current_node.y) > 1
        out = sqrt(2);
    else
        out = 1;
    end
end