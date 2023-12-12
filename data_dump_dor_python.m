tree = load ("imgs/tree/path_1round_1.mat").tree;
tree_nodes = load("imgs/tree/path_1round_1.mat").valid_nodes;

tree_nodes_mat = cell2mat(tree_nodes);

elementsPerRow = 6;

numRows = numel(tree_nodes_mat) / elementsPerRow;

newMat = reshape(tree_nodes_mat, elementsPerRow, numRows)';

out_tree_path='csv/tree.csv';
out_node_path='csv/nodes.csv';

csvwrite(out_tree_path, tree);
csvwrite(out_node_path, newMat);


