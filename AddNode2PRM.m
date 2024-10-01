function out = AddNode2PRM (x, prm, neighbors, obstacle)
% Add a node to a PRM - used to attach start and destination configurations

distances = dist(x, prm.samples);

[sorted_distances, idx] = sort(distances);

n = length(distances);

edges = zeros(neighbors,2);
edge_lengths = zeros(neighbors,1);

nedges = 0;

for i = 1:min(neighbors, n)
    j = idx(i);
    if  ~obstacle(x(1),x(2))
        nedges = nedges + 1;
        edges(nedges,:) = [n+1, j];
        edge_lengths(nedges) = sorted_distances(i);
    end
end

out.samples = [prm.samples, x(:)];
out.edges = [prm.edges; edges(1:nedges, :)];
out.edge_lengths = [prm.edge_lengths; edge_lengths(1:nedges)];