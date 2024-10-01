function route = ShortestPathDijkstra(edges, edge_lengths, samples, start_sample, goal_sample)

    N = goal_sample;
    % Construct distance map
    map = Inf(N,N);
    for i = 1:length(edges(:,1))
        map(edges(i,1), edges(i,2)) = edge_lengths(i);
        map(edges(i,2), edges(i,1)) = edge_lengths(i);
    end
    % Make diagonal 0
    for i = 1:N
        map(i, i) = 0;
    end
    % Parent array to store route order
    parent = zeros(1,N);

    distances(1:N) = Inf;
    visited(1:N) = 0;
    distances(start_sample) = 0; % Saves the distance from the start point to the rest of the points

    while sum(visited) < N
        candidates(1:N) = Inf;
        for index = 1:N
            if visited(index) == 0
                candidates(index) = distances(index);
            end
        end
        [cur_d, cur] = min(candidates);
        
        % Assign parent path when first visit this node
        % % Only assign parents to those who don't have parents
        for i = 1:N
            if map(cur,i) ~= Inf && parent(i) == 0
                parent(i) = cur;
            end
        end
        for index = 1:N
            newDistance = cur_d + map(cur, index);
            if newDistance < distances(index)
                distances(index) = newDistance;
                parent(index) = cur; % Update parent path because the new path is shorter
            end
        end
        visited(cur) = 1;
    end

    route = samples(:,goal_sample)';
    idx = goal_sample;
    count = 0;
    while samples(:,idx)' ~=  samples(:,start_sample)'
        idx = parent(idx);
        route = [samples(:,idx)';route];
        count = count + 1;
    end
end