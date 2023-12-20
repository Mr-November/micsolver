function [optpath, totcost] = dp(xisc, lossfun)
%DP Finds the shortest path in a graph.
%   [PATH, COST] = DP(XISC, LOSSFUN) returns the paths and corresponding
%   costs using the Dijkstra s algorithm. The cell array XISC defines the
%   vertices. The function handle LOSSFUN defines the weight of two
%   adjacent edges. The output PATH and COST are cell arrays.

n1 = size(xisc{1}, 2);
pre_cost = zeros(n1, 1);
cur_cost = NaN(n1, 1);
pre_path = num2cell((1: n1).');

for k = 1: size(xisc, 2)-1
    m = size(xisc{k+1}, 2);
    cost = NaN(m, 1);
    path = num2cell(NaN(m, 1));
    for j = 1: m
        for i = 1: size(xisc{k}, 2)
            cur_cost(i) = pre_cost(i) + lossfun(xisc{k}(:, i), xisc{k+1}(:, j));
        end
        cost(j) = min(cur_cost);
        idcs = find(cur_cost == cost(j));
        path{j} = [vertcat(pre_path{idcs}), j .* ones(size(vertcat(pre_path{idcs}), 1), 1)];
    end
    pre_cost = cost;
    cur_cost = NaN(size(pre_cost));
    pre_path = path;
end

totcost = pre_cost;
optpath = pre_path;

end


