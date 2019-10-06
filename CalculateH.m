function val = CalculateH(node)
global end_index
val = abs(end_index(1) - node(1)) + abs(end_index(2) - node(2)) + abs(end_index(3) - node(3)) + randn * randn * 0.01;
end