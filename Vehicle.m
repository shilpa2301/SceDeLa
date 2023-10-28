%note: need to intelligently decide the steering angle, same angle+ noise
%dont work
%iteration wise trapezium wont work
%need to stop car making rounds in the road - check dist calc from current
%trapezium borders
%termination point for car as thr road end, not iteration
classdef Vehicle
   properties

       waypoints 
       speed 
       iteration
       last_road_block
      
   end

   methods
       function veh_obj=Vehicle(scenario)

            
            veh_obj.iteration=1;

            centers = scenario.RoadSpecifications.Centers;

            
            %get road boundaries
            new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
            laneSpecification = lanespec([1 1]);
            road(new_scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
            rbScenario = roadBoundaries(new_scenario_obj);

            border_total=rbScenario{1,1};
            if mod(border_total,2)==0
                first_border_end=floor(length(border_total)/2);
                second_border_start=first_border_end+1;
            else
                first_border_end=floor(length(border_total)/2);
                second_border_start=first_border_end+2;
            end
            border_1=border_total(1:first_border_end, :);
            border_2=flip(border_total(second_border_start:length(border_total), :));
            
       end   
        
       function dist=dist_func(veh_obj, x1,y1, x2,y2)
            dist=sqrt((x1-x2)^2 + (y1-y2)^2);
       end

       function [min_node_id, min_dist]=find_nearest_node_in_tree(veh_obj, valid_nodes_list, x2, y2)
            min_dist=Inf;
            min_node_id=-1;
            for i = 1: width(valid_nodes_list)
                node=valid_nodes_list(i);
                %node_y=valid_nodes_list(i,2);
                dist=dist_func(veh_obj, node{1}(1), node{1}(2), x2, y2);
                if dist<min_dist
                    min_dist=dist;
                    min_node_id=i;

                end

            end
            
       end

       function [sortednodelist, min_dist_list] = find_k_nearest_node_in_tree(veh_obj, valid_nodes_list, x2, y2,k)
           nodelist = [];
           min_dist_list=[];
           for i = 1: width(valid_nodes_list)
                node=valid_nodes_list(i);
                %node_y=valid_nodes_list(i,2);
                dist=dist_func(veh_obj, node{1}(1), node{1}(2), x2, y2);
                
                nodelist(end+1)=dist;
                min_dist_list(end+1)=i;               

           end

           

           [min_dist_list, sortednodelist] = sort(nodelist);
           if length(nodelist)>=k
                min_dist_list=min_dist_list(1:k);
                sortednodelist=sortednodelist(1:k);
           end


       end

       
       
       function [targetx,targety] = find_target_point(veh_obj, x1,y1, x2,y2, dist_of_new_node)
            m = (y2-y1)/(x2-x1);
            c=y1- m*x1;

            syms x3 y3
            eqns=[y3==m*x3+c, sqrt((x3-x1)^2 + (y3-y1)^2)==dist_of_new_node];
            S=solve(eqns, [x3 y3]);

            if(dist_func(veh_obj,S.x3(1),S.y3(1), x2, y2) <= dist_func(veh_obj,S.x3(2),S.y3(2), x2, y2) )
                targetx=S.x3(1);
                targety=S.y3(1);
            else
                targetx=S.x3(2);
                targety=S.y3(2);
            end


       end

       % calculates the distance between a dot and a line
       function dist = point_to_line(veh_obj,pt, v1, v2)
            a = v1 - v2;
            b = pt - v2;
            line_vec = a ;%vector(start, end) # (3.5, 0, -1.5)
            pnt_vec = b ;%vector(start, pnt)  # (1, 0, -1.5)
            line_len = sqrt(sum(line_vec.^2)); % # 3.808
            line_unitvec = line_vec/line_len; % # (0.919, 0.0, -0.394)
            pnt_vec_scaled = pnt_vec/line_len; %  # (0.263, 0.0, -0.393)
            t = dot(line_unitvec, pnt_vec_scaled); % # 0.397
            if t < 0.0
                t = 0.0;
            elseif t > 1.0
                t = 1.0;
            end
            nearest = line_vec* t; %    # (1.388, 0.0, -0.595)
            dist = sqrt(sum((nearest-pnt_vec).^2));% # 0.985
            nearest = nearest+v2;
       end
        
       %remember to do for only 1 last block, not 2 blocks which is done in
       %general
       function [validity_flag, corresp_road_block]=find_if_node_inside_any_road_block(veh_obj, target_x, target_y, border_1, border_2,block_threshold, nearest_node_block_id)
            validity_flag=false;
            corresp_road_block = -1;
            %distance_rom_block_end=-1;
            %current_road_block= current_road_blocks;
            start_block= 1;
            end_block =start_block+block_threshold;
            for bt = 2:-1:1
                if nearest_node_block_id - bt>=1
                    start_block = nearest_node_block_id - bt;
                    break
                end
            end

            for bt = block_threshold:-1:1
                if nearest_node_block_id + bt<=length(border_1)
                    end_block = nearest_node_block_id + bt;
                    break
                end
            end
           
            for current_road_block=start_block:end_block
            
                    left_bottom_x=border_1(current_road_block,1);
                    left_bottom_y=border_1(current_road_block,2);
                    left_top_x   =border_2(current_road_block,1);
                    left_top_y   =border_2(current_road_block,2);
                    if current_road_block+block_threshold<=length(border_1)
                        end_check = current_road_block+block_threshold;
                    else
                        end_check = length(border_1);
                    end
                    right_bottom_x=border_1(end_check,1);
                    right_bottom_y=border_1(end_check,2);
                    right_top_x   =border_2(end_check,1);
                    right_top_y   =border_2(end_check,2);
                    
                   
        
                    %distance_from_block_end=-1;
                    corresp_road_block = -1;
        
                    if inpolygon(target_x,target_y,[left_bottom_x right_bottom_x right_top_x left_top_x], ...
                            [left_bottom_y right_bottom_y right_top_y left_top_y])
                        validity_flag=1;
        
                        %calculate distance from block end
                        pt=[target_x target_y];
                        v1=[right_bottom_x right_bottom_y];
                        v2=[right_top_x right_top_y];
                        corresp_road_block = current_road_block;
                       
                        break
        
                    end
            end
%
       
       end
          
            function [validity_flag, corresp_road_block]=find_if_target_node_valid(veh_obj, target_x, target_y, border_1, border_2,block_threshold, nearest_node_block_id)

            validity_flag=false;
            corresp_road_block = -1;
            
            [validity_flag,corresp_road_block ]=find_if_node_inside_any_road_block(veh_obj, target_x, target_y, border_1, border_2,block_threshold, nearest_node_block_id);
            

       end

       function waypoints_total=sampled_scenario(veh_obj, scenario)

           global last_road_block;
           last_road_block = 0;
           centers = scenario.RoadSpecifications.Centers;
           new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
           laneSpecification = lanespec([1 1]);
           road(new_scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
           rbScenario = roadBoundaries(new_scenario_obj);

           border_total=rbScenario{1,1};
           if mod(border_total,2)==0
               first_border_end=floor(length(border_total)/2);
               second_border_start=first_border_end+1;
           else
               first_border_end=floor(length(border_total)/2);
               second_border_start=first_border_end+2;
           end
           border_1=border_total(2:first_border_end, :);
           border_2=flip(border_total(second_border_start:length(border_total)-1, :));
            
           %dist_from_block_end=1.0;
           dist_of_new_node=3.0;
           %current_road_block=1;
           tree=dictionary;
           valid_nodes={[-160 -28.2 -0.63 0 1]}; % [x y heading steering_angle blockid] and -pi/5 = -0.63 = 36 degrees
           flag_distant_from_other_points = 0;
           threshold_dist_from_other_points = 1.0;%1.0;
           block_threshold = 6; %with dist = 1.0, this val = 6
           knn=20;
           velocity_change_threshold = 2.7; % 10km/hr per frame
           steer_angle_rate_change_threshold = 0.5; % 0.5 radian = 30 degrees % 0.17; %10 degrees in radians
           input_velocity = 27.8; % 100 km/h = 27.8 m/s; 12.5; %45 km/hr
           input_steer_angle_rate = 0.0; % driving straight
           L=0.5;
           delta_t=0.05;

           corresp_road_block = 1;

           while corresp_road_block<length(border_1)-block_threshold
               %disp(current_road_block);
               %sample a
                sampled_x=-240+ ((-40)-(-240))*rand; %((-300)+rand*(300-(-300)));
                sampled_y=-90 + ((-20)-(-90))*rand; %((-300)+rand*(300-(-300));
               
               
               
               %Find if the point's distance id < delta_distance=1.0
                        [nearest_node_list, dist_list] = find_k_nearest_node_in_tree(veh_obj, valid_nodes, sampled_x, sampled_y,knn);
                        %[nearest_node_id, dist_from_curr_state]=find_nearest_node_in_tree(veh_obj, valid_nodes, sampled_x, sampled_y);
                        for nn = 1:length(nearest_node_list)
                        %dist_from_curr_state=dist_func(veh_obj, state(end,1), state(end,2), sampled_x, sampled_y);
                            nearest_node_id = nearest_node_list(nn);
                            dist_from_curr_state = dist_list(nn);
                            disp(["checking with nearest neighbor",num2str(nn)])
                            %if yes -> this is the next way point
                            %if dist_from_curr_state<=dist_of_new_node %1.5=theshold distance to create node
                            %    %do nothing
%
                            %    %target_x = sampled_x;
                            %    %target_y= sampled_y;
    %%
                            %    %%shilpa- fix to get whole row
                            %    %start_node = valid_nodes(nearest_node_id);
                            %
                            %% if no, find target point connecting the current
                            %% state and sampled state
                            %else
                                start_node = valid_nodes(nearest_node_id);
                                [target_x,target_y] = find_target_point(veh_obj, start_node{end}(1), start_node{end}(2), sampled_x, sampled_y,dist_of_new_node);
                                target_x=double(target_x);
                                target_y=double(target_y);
    
                                if (target_x ~= -1) || (target_y ~= -1)

                                    %sampled points and bicycle model
                                    sampled_steering_angle_rate_changed = zeros(1,10);
                                    sampled_velocity_change = zeros(1,10);
                                    for ss = 1:10
                                    sampled_steering_angle_rate_changed(ss)=((-1)*steer_angle_rate_change_threshold)+ (steer_angle_rate_change_threshold-((-1)*steer_angle_rate_change_threshold))*rand; % 30 degrees
                                    sampled_velocity_change(ss)=((-1)*velocity_change_threshold) + (velocity_change_threshold-((-1)*velocity_change_threshold))*rand; %10 km/hr change
                                    end

                                    %calculating current inputs to model
                                    %based on change
                                    states = zeros(100,4);
                                    itss=1;
                                    for ang_ch = 1:10
                                        for vel_ch = 1:10
                                            input_velocity = input_velocity + sampled_velocity_change(vel_ch);
                                            input_steer_angle_rate = input_steer_angle_rate + sampled_steering_angle_rate_changed(ang_ch);
        
                                            %bicycle model calculations
        
                                            %considering rear wheel axle
                                            %x_dot= input_vel*cos(state(i-1,3));
                                            %y_dot= input_vel*sin(state(i-1,3));
                                            %%rear wheel
                                            %R= L/tan(state(i-1,4));
                                            %omega=input_vel/R;
                                            %theta_dot=(input_vel*tan(state(i-1,4)))/L;
                                            %steer_dot=input_steer_angle_rate;
                            
                                            %considering front wheel axle
                                            x_dot= input_velocity*cos(valid_nodes{nearest_node_id}(3)+valid_nodes{nearest_node_id}(4));
                                            y_dot= input_velocity*sin(valid_nodes{nearest_node_id}(3)+valid_nodes{nearest_node_id}(4));
                                            %front wheel
                                            R=L/sin(valid_nodes{nearest_node_id}(4));
                                            omega=input_velocity/R;
                                            theta_dot=(input_velocity*sin(valid_nodes{nearest_node_id}(4)))/L;
                                            steer_dot=input_steer_angle_rate;
                                            
                                            
                                            x_next=valid_nodes{nearest_node_id}(1) + x_dot*delta_t;
                                            y_next=valid_nodes{nearest_node_id}(2) + y_dot*delta_t;
                                            theta_next=valid_nodes{nearest_node_id}(3) + theta_dot*delta_t;
                                            steer_next=valid_nodes{nearest_node_id}(4) + steer_dot*delta_t;
                                            states(itss,:)=[x_next y_next theta_next steer_next];
                                            itss=itss+1;
                                        end
                                    end
        
                                    %sort as per dist from target_x,
                                    %target_y
                                    nodelist = [];
                                    min_dist_list=[];
                                    for itss = 1:100 
                                        
                                        
                                             node=states(itss,:);
                                             %node_y=valid_nodes_list(i,2);
                                             dist=dist_func(veh_obj, node(1), node(2), target_x, target_y);
                                        
                                             nodelist(end+1)=dist;
                                             min_dist_list(end+1)=i;               
                                
                                   end                             
                                   [out, sorted_idx_list] = sort(nodelist);
                                                                     
                                    
                                    for itss = 1:100
                                            %if the node already doesnt exist then consider
                                            %it for adding - takes care of the DS being a
                                            %tree instead of a graph95701344014
                                            idx = sorted_idx_list(itss);
                                            curr_state = states(idx,:);
                                            considered_x = curr_state(1);
                                            considered_y = curr_state(2);
                                            considered_head = curr_state(3);
                                            considered_steer = curr_state(4);
                                    
                                            index=0;
                                            for n=1:length(valid_nodes)
                                                if considered_x == valid_nodes{n}(1) && considered_y == valid_nodes{n}(2)
                                                    index=n;
                                                    break;
                                                end
                                            end
                                    
                                            if index == 0
                                                flag_distant_from_other_points = 1;
                                                for n = 1:length(valid_nodes)
                                                    dist_from_n = dist_func(veh_obj,considered_x, considered_y, ...
                                                        valid_nodes{n}(1), valid_nodes{n}(2));
                                                    if dist_from_n <= threshold_dist_from_other_points
                                                        flag_distant_from_other_points = 0;
                                                    end
            
                                                end
                                              if flag_distant_from_other_points ==0 
            
                                                disp("distance from other nodes very less")  
                                              end
                                            end    

                                            if flag_distant_from_other_points == 1
                                            %if ismember([target_x target_y], valid_nodes) == false
                                               %if target point inside road -> this is the next
                                                %waypoint
                                                %else discard this point and sample again
                                                [validity_flag, corresp_road_block]=find_if_target_node_valid(veh_obj, considered_x, considered_y, border_1, border_2,block_threshold, valid_nodes{nearest_node_id}(5));
                                                if validity_flag == true
                                                    disp("New point found")
                                                    valid_nodes{end+1} = [considered_x considered_y considered_head considered_steer corresp_road_block];
                                                    if last_road_block<corresp_road_block
                                                        last_road_block = corresp_road_block;
                                                    end
        
                                                    tree(nearest_node_id) = length(valid_nodes);
                
                                                    node_x=[];
                                                    node_y=[];
                                                    for l=1:length(valid_nodes)
                                                         node_x(end+1)=valid_nodes{l}(1);
                                                         node_y(end+1)=valid_nodes{l}(2);
                                                    end
                                    
                                                    scatter(border_1(:,1), border_1(:,2), 'red', "filled");
                                                    hold on 
                                                    scatter(border_2(:,1), border_2(:,2), 'red', "filled");
                                                    hold on
                                                    scatter(node_x, node_y, 'blue', "filled");
                                                    saveas(gcf,'imgs/trial/'+string(length(valid_nodes)) + '.png')
                                                    close
                                                    break
                                                end
                                            
                                            %check dist from each node <0.5/1.0    
                                            end
                                            %might need to break here for
                                            %100 sampled points

                                        end
                                   
                                end
                           % end

                        end


           end
           
           node_x=[];
           node_y=[];
           for l=1:length(valid_nodes)
                node_x(end+1)=valid_nodes{l}(1);
                node_y(end+1)=valid_nodes{l}(2);
           end

           scatter(border_1(:,1), border_1(:,2), 'red', "filled");
           hold on 
           scatter(border_2(:,1), border_2(:,2), 'red', "filled");
           hold on
           scatter(node_x, node_y, 'blue', "filled");


           %comment out later- currently visualize RRT on road
           scenario_obj = drivingScenario('VerticalAxis', 'Y');
              
           % Add road data (assuming road_data is a field in the struct)
           %scenario_obj.road = scenario_with_ego_veh.scenario_inst.RoadSpecifications;
           %scene=drivingScenarioDesigner(DSD_Scenario
           laneSpecification = lanespec([1 1]);
           road(scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive')

           
           % Plot the driving scenario
           plot(scenario_obj, 'Centerline','on','RoadCenters','on','Waypoints','on');

           pi=2;
           



       end



     

       function veh_obj=vehicle_movement(veh_obj, scenario)%iteration, waypoints, speed)

           %rbScenario = roadBoundaries();
           new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
           laneSpecification = lanespec([1 1]);
           road(new_scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
           rbScenario = roadBoundaries(new_scenario_obj);

           waypoints_total=sampled_scenario(veh_obj, scenario);
          speed_total = ones(1, length(waypoints_total))*30;

          while veh_obj.iteration<length(waypoints_total)-1
            veh_obj.iteration=veh_obj.iteration+1;
            next_position=waypoints_total(veh_obj.iteration, :);
            veh_obj.waypoints=[veh_obj.waypoints; next_position];
            next_speed=speed_total(veh_obj.iteration);
            veh_obj.speed=[veh_obj.speed; next_speed];
          end
           
       end

       

   end
end