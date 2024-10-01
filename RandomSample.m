function sample = RandomSample (obstacle)
% Generate a random freespace configuration for the robot

while true
    s = size(obstacle);

    x = ceil(s(2)*rand(1));
    y = ceil(s(1)*rand(1));
    sample = [y, x];
    if (~obstacle(y,x))
        return
    end
end
