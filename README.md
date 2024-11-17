# SceDeLa
Scenario Description Language in Autonomous Vehicles

A. Running Code to generate instances:
2. 1. Create the folder imgs/trial/ in the same folder as main.m
   2. Uncomment sampled_scenario(veh_obj, scenario, veh_id, num_paths); in vehicle_movement function of Vehicle.m
   3. Comment the following lines in the same function:
           tree = load ("imgs/tree/path_1round_1.mat").tree;
           tree_nodes = load("imgs/tree/path_1round_1.mat").valid_nodes;
           waypoint_idx = find_path(veh_obj, scenario, tree, tree_nodes, veh_id);         

           for ii=1:length(waypoint_idx)
                veh_obj.iteration = ii;
                x=tree_nodes{waypoint_idx(ii)}(1);
                y=tree_nodes{waypoint_idx(ii)}(2);
                veh_obj.waypoints=[veh_obj.waypoints; [x y]];
                veh_obj.speed=[veh_obj.speed; 50];
           end
   
   5.In order to visualize each generated point, uncomment the below lines inside "if validity_flag == true" block in function sampled_scenario:
     % scatter(border_1(:,1), border_1(:,2),5, 'red', "filled");
    % hold on 
     % scatter(border_2(:,1), border_2(:,2),5, 'red', "filled");
     % hold on
     % scatter(node_x, node_y, 5, 'blue', "filled");
     % % saveas(gcf,'imgs/trial/path'+string(p)+'_'+string(length(valid_nodes)) + '.png')
      % close
   6. Run main.m


   B. GENERATING and TRAVERSING THE DUMPED TREES TO FIND A PATH
    1. Comment sampled_scenario(veh_obj, scenario, veh_id, num_paths); in vehicle_movement function of Vehicle.m
   2. Uncomment the following lines in the same function (make sure to use your tree names from the folder:
           tree = load ("imgs/tree/path_1round_1.mat").tree;
           tree_nodes = load("imgs/tree/path_1round_1.mat").valid_nodes;
           waypoint_idx = find_path(veh_obj, scenario, tree, tree_nodes, veh_id);         

           for ii=1:length(waypoint_idx)
                veh_obj.iteration = ii;
                x=tree_nodes{waypoint_idx(ii)}(1);
                y=tree_nodes{waypoint_idx(ii)}(2);
                veh_obj.waypoints=[veh_obj.waypoints; [x y]];
                veh_obj.speed=[veh_obj.speed; 50];
           end

   C. Generating Data structure for Model checker in Python:
   1. Run data_dump_dor_python.m with correct file names
   2. Run main.py with correct file names


This work is published at Runtime Verification 2023. If you are referring to this work, please cite:

@inproceedings{kudalkar2024sampling,
  title={Sampling-Based and Gradient-Based Efficient Scenario Generation},
  author={Kudalkar, Vidisha and Hashemi, Navid and Mukhopadhyay, Shilpa and Mallick, Swapnil and Budnik, Christof and Nagaraja, Parinitha and Deshmukh, Jyotirmoy V},
  booktitle={International Conference on Runtime Verification},
  pages={70--88},
  year={2024},
  organization={Springer}
}
