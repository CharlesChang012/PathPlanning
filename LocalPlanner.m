function out = LocalPlanner (x, new, obstacle)
% Generate a random freespace configuration for the robot
% x : (y, x)
    [m, b] = str_lin1(x(2), x(1), new(2), new(1));
    
    for i = 1:length(obstacle(:,1)) % y direction
        for j = 1:length(obstacle(1,:)) % x direction
            if obstacle(i,j)
                y = m*j + b;
                if x(1) <= j && new(1) >= j && i == round(y)
                    out = false;
                    return
                elseif x(1) >= j && new(1) <= j && i == y
                    out = false;
                    return
                end
            end
        end
    end
    out = true; 
end



