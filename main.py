import pandas as pd

# Step 1: Read the first CSV with parent-child relationships
df_relations = pd.read_csv('~/Shilpa/work/Internship_summer23/Scripts/work1/csv/tree.csv', header=None,
                           names=['parent', 'child'])

# Step 2: Read the second CSV with node details
df_nodes = pd.read_csv('~/Shilpa/work/Internship_summer23/Scripts/work1/csv/nodes.csv', header=None,
                       names=['x', 'y', 'heading', 'steering', 'blockid', 'laneid'])


# Create a class to represent each node in the tree
class TreeNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.parent = parent
        self.children_list = []


# Create a dictionary to store nodes based on their state
nodes_dict = {}

# Initialize the root node with details from the first row of 'nodes.csv'
root_state = {'x': df_nodes.loc[0, 'x'], 'y': df_nodes.loc[0, 'y'], 'heading': df_nodes.loc[0, 'heading'],
              'steering': df_nodes.loc[0, 'steering'], 'blockid': df_nodes.loc[0, 'blockid'],
              'laneid': df_nodes.loc[0, 'laneid']}
root_node = TreeNode(root_state, parent=None)
nodes_dict[0] = root_node  # Use -1 as the key for the root node

# Populate nodes_dict with nodes based on node details
for index, row in df_nodes.iloc[1:].iterrows():
    state = {'x': row['x'], 'y': row['y'], 'heading': row['heading'], 'steering': row['steering'],
             'blockid': row['blockid'], 'laneid': row['laneid']}
    node = TreeNode(state)
    nodes_dict[index] = node

# Populate the tree structure based on parent-child relationships
for index, row in df_relations.iterrows():
    parent_index = row['parent']
    child_index = row['child']

    parent_node = nodes_dict[parent_index-1]
    child_node = nodes_dict[child_index-1]

    # Update child's parent and add child to parent's children_list
    child_node.parent = parent_node
    parent_node.children_list.append(child_node)

# # Function to print the tree in a readable format
# def print_tree(node, level=0):
#     print("  " * level + str(node.state))
#     for child in node.children_list:
#         print_tree(child, level + 1)
#
# # Print the tree starting from the root node
# print_tree(root_node)

a=2
