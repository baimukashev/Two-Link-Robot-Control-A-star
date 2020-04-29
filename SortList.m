function sortedS = SortList( list )
    T = struct2table(list); % convert the struct array to a table
    sortedT = sortrows(T, 'est_cost'); % sort the table by 'past_code'
    sortedS = table2struct(sortedT);     
end