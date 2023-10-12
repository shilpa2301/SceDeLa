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
            for i = 1: height(valid_nodes_list)
                node=valid_nodes_list(i);
                %node_y=valid_nodes_list(i,2);
                dist=dist_func(veh_obj, node{1}(1), node{1}(2), x2, y2);
                if dist<min_dist
                    min_dist=dist;
                    min_node_id=i;

                end

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
                targetx=-1;%S.x3(2);
                targety=-1;%S.y3(2);
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
       function [validity_flag, distance_from_block_end]=find_if_node_inside_any_road_block(veh_obj, current_road_blocks, target_x, target_y, border_1, border_2)
            validity_flag=false;
            distance_rom_block_end=-1;

            for current_road_block=1:current_road_blocks+2
            
                    left_bottom_x=border_1(current_road_block,1);
                    left_bottom_y=border_1(current_road_block,2);
                    left_top_x   =border_2(current_road_block,1);
                    left_top_y   =border_2(current_road_block,2);
                    right_bottom_x=border_1(current_road_block,1);
                    right_bottom_y=border_1(current_road_block,2);
                    right_top_x   =border_2(current_road_block,1);
                    right_top_y   =border_2(current_road_block,2);
                    
                   
        
                    distance_from_block_end=-1;
        
                    if inpolygon(target_x,target_y,[left_bottom_x right_bottom_x+2 right_top_x+2 left_top_x], ...
                            [left_bottom_y right_bottom_y right_top_y left_top_y])
                        validity_flag=1;
        
                        %calculate distance from block end
                        pt=[target_x target_y];
                        v1=[right_bottom_x right_bottom_y];
                        v2=[right_top_x right_top_y];
                        if current_road_block == current_road_blocks
                            distance_from_block_end = point_to_line(veh_obj, pt, v1, v2);
                        end
                        break
        
                    end
            end
%
            %if ((target_x>=left_bottom_x && target_x<=left_top_x) ...
            %        ||(target_x<=left_bottom_x && target_x>=left_top_x)) && ...
            %    ((target_x>=right_bottom_x && target_x<=right_top_x) ...
            %        ||(target_x<=right_bottom_x && target_x>=right_top_x)) && ...
            %        ((target_y>=left_bottom_y && target_y<=left_top_y) ...
            %        ||(target_y<=left_bottom_y && target_y>=left_top_y)) && ...
            %            ((target_y>=right_bottom_y && target_y<=left_top_y) ...
            %            ||(target_y<=left_bottom_y && target_y>=left_top_y))
       end

       function [validity_flag, current_road_block]=find_if_target_node_valid(veh_obj, dist_from_block_end, current_road_block, target_x, target_y, border_1, border_2)

            validity_flag=false;
            if current_road_block+2<length(border_1)
                    [validity_flag, distance_rom_block_end]=find_if_node_inside_any_road_block(veh_obj, current_road_block, target_x, target_y, border_1, border_2);
            end

            if distance_rom_block_end<=dist_from_block_end && distance_rom_block_end ~= -1%0.5=threshold from road block boundary to end
                current_road_block=current_road_block+1;
            end


       end

       function waypoints_total=sampled_scenario(veh_obj, scenario)
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
            
           dist_from_block_end=1.0;
           dist_of_new_node=4.0;
           current_road_block=1;
           tree=dictionary;
           valid_nodes={[-156.0 -28.2]};
           flag_distant_from_other_points = 0;
           threshold_dist_from_other_points = 0.7;

           while current_road_block<length(border_1)-2
               disp(current_road_block);
               %sample a
               sampled_x=-240+ ((-40)-(-240))*rand; %((-300)+rand*(300-(-300)));
               sampled_y=-90 + ((-20)-(-90))*rand; %((-300)+rand*(300-(-300)));
               
               %Find if the point's distance id < delta_distance=1.0
                        [nearest_node_id, dist_from_curr_state]=find_nearest_node_in_tree(veh_obj, valid_nodes, sampled_x, sampled_y);
                        
                        %dist_from_curr_state=dist_func(veh_obj, state(end,1), state(end,2), sampled_x, sampled_y);
                       
                        %if yes -> this is the next way point
                        if dist_from_curr_state<=dist_of_new_node %1.5=theshold distance to create node
                            target_x = sampled_x;
                            target_y= sampled_y;

                            %shilpa- fix to get whole row
                            start_node = valid_nodes(nearest_node_id);
                            
                        % if no, find target point connecting the current
                        % state and sampled state
                        else
                            start_node = valid_nodes(nearest_node_id);
                            [target_x,target_y] = find_target_point(veh_obj, start_node{end}(1), start_node{end}(2), sampled_x, sampled_y,dist_of_new_node);
                            target_x=double(target_x);
                            target_y=double(target_y);

                            if (target_x ~= -1) || (target_y ~= -1)
                                
                            
                            
                                %if the node already doesnt exist then consider
                                %it for adding - takes care of the DS being a
                                %tree instead of a graph
                                
                                index=0;
                                for n=1:length(valid_nodes)
                                    if target_x == valid_nodes{n}(1) && target_y == valid_nodes{n}(2)
                                        index=n;
                                        break;
                                    end
                                end
                                
                                if index == 0
                                    flag_distant_from_other_points = 1;
                                    for n = 1:length(valid_nodes)
                                        dist_from_n = dist_func(veh_obj,target_x, target_y, ...
                                            valid_nodes{n}(1), valid_nodes{n}(2));
                                        if dist_from_n <= threshold_dist_from_other_points
                                            flag_distant_from_other_points = 0;
                                        end

                                    end
                                  if flag_distant_from_other_points ==0 

                                    disp("distance from other nodes very less")  
                                  end
                                end    
                                %index = find([valid_nodes{:}] == [target_x target_y]);
                                if flag_distant_from_other_points == 1
                                %if ismember([target_x target_y], valid_nodes) == false
                                   %if target point inside road -> this is the next
                                    %waypoint
                                    %else discard this point and sample again
                                    [validity_flag, current_road_block]=find_if_target_node_valid(veh_obj, dist_from_block_end, current_road_block, target_x, target_y, border_1, border_2);
                                    if validity_flag == true
                                        valid_nodes{end+1} = [target_x target_y];
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
                                        saveas(gcf,'imgs/'+string(length(valid_nodes)) + '.png')
                                        close
                                    end
                                
                                end
                            end
                        end




           end
           
           node_x=[];
           node_y=[];
           for l=1:length(valid_nodes)
                node_x(end+1)=valid_nodes{l}(1);
                node_y(end+1)=valid_nodes{l}(2);
           end

           scatter(border_1(:,1), border_1(:,2));
           hold on 
           scatter(border_2(:,1), border_2(:,2));
           hold on
           scatter(node_x, node_y);


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