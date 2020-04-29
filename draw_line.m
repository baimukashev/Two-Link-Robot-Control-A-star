% draw a two link 
function h = draw_line(x1,y1,x2,y2)
hold on
plot([0 x1], [0 y1], 'b');
hold on
plot([x1 x2], [y1 y2], 'b');
%pause(0.01)
hold on
end