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

            %Random initialization of position
            %mid_road=scenario.RoadSpecifications.Centers(1, :);
%
            %
            %lane_width_size=length(scenario.RoadSpecifications.Lanes.Width);
            %mid_lane=lane_width_size/2.0;
%
            %%calculate left most point or road start
            %if lane_width_size%2==0
            %    ln=mid_lane;
            %    left_width_sum=0;
            %    while ln>0
            %        left_width_sum=left_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
            %        ln=ln-1;
            %    end
            %else
            %    ln=ceil(mid_lane);
            %    left_width_sum=scenario.RoadSpecifications.Lanes.Width(1,ln)/2.0;
            %    ln=ln-1;
            %    while ln>0
            %        left_width_sum=left_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
            %        ln=ln-1;
            %    end
            %end
            %left_boundary=mid_road(1,1) - left_width_sum;
%
%
            % %calculate right most point or road start
            % if lane_width_size%2==0
            %    ln=mid_lane;
            %    right_width_sum=0;
            %    while ln<=lane_width_size
            %        right_width_sum=right_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
            %        ln=ln+1;
            %    end
            %else
            %    ln=ceil(mid_lane);
            %    right_width_sum=scenario.RoadSpecifications.Lanes.Width(1,ln)/2.0;
            %    ln=ln+1;
            %    while ln>0
            %        right_width_sum=right_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
            %        ln=ln+1;
            %    end
            %end
            %right_boundary=mid_road(1,1) + right_width_sum;

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
            
            %
            

             %sample a point between boundary at the start of road
            %x_val=left_boundary + rand*(right_boundary-left_boundary);

            %veh_obj.waypoints= [x_val mid_road(1,2) mid_road(1,3)];
            veh_obj.waypoints=[border_1(1,1)-1 border_1(1,2)-1 0];
            %Veh_obj.waypoints =[-161.9 -28.2 0];

            veh_obj.speed=[30];
            
            
          
            
       end

       %function [left_boundary, right_boundary]=get_road_boundaries(scenario, i)
       %end

       %function waypoints_total=kinematic_bicycle_model(veh_obj, scenario, referenceWayPoints)
       %    kinematicModel = bicycleKinematics;
       %    initialState = veh_obj.waypoints ;%[0 0 0];
       %    road_waypoint_count=length(scenario.RoadSpecifications.Centers);
       %    tspan = 1:1:road_waypoint_count;
%      %     inputs= [(0.1 + rand*(60-0.1)) (-pi/4 + rand*(pi/4-(-pi/4)))];
       %    states = zeros(size(referenceWayPoints,1),3);
       %    inputs = zeros(size(refernceWayPoints,1)-1,2);
       %    inputs(1,:) = [(0.1 + rand*(60-0.1)) (-pi/4 + rand*(pi/4-(-pi/4)))];
       %    for jj=2:size(referenceWayPoints,1)
       %        cFn = @(in) costFunction(referenceWayPoints(jj,:),kinematics,states(jj-1,:),0:.05:1,in);
       %        inputToUse = fminsearch(@costFunction, inputs(jj-1,:));
       %        inputs(jj,:) = inputsToUse;
       %    end
%
       %    %inputs = [30 -pi/4]; %Turn left
%
       %    [t,y] = ode45(@(t,y) derivative(kinematicModel,y,inputs),tspan,initialState);
       %    waypoints_total=y;
       %end
           

       %function waypoints_total=generate_waypoints(veh_obj, scenario)
       %   i=1;
       %   road_waypoint_count=length(scenario.RoadSpecifications.Centers);
       %   waypoints_total= veh_obj.waypoints;
       %   
       %   while i<road_waypoint_count
       %         %Random initialization of position
       %     i=i+1;
       %     %[left_boundary, right_boundary]=get_road_boundaries(scenario, i);
%
       %      mid_road=scenario.RoadSpecifications.Centers(i, :);
%
       %     
       %     lane_width_size=length(scenario.RoadSpecifications.Lanes.Width);
       %     mid_lane=lane_width_size/2.0;
%
       %     %calculate left most point or road start
       %     if mod(lane_width_size,2)==0
       %         ln=mid_lane;
       %         left_width_sum=0;
       %         while ln>0
       %             left_width_sum=left_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
       %             ln=ln-1;
       %         end
       %     else
       %         ln=ceil(mid_lane);
       %         left_width_sum=scenario.RoadSpecifications.Lanes.Width(1,ln)/2.0;
       %         ln=ln-1;
       %         while ln>0
       %             left_width_sum=left_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
       %             ln=ln-1;
       %         end
       %     end
       %     left_boundary=mid_road(1,1) - left_width_sum;
%
%
       %      %calculate right most point or road start
       %      if mod(lane_width_size,2)==0
       %         ln=mid_lane;
       %         right_width_sum=0;
       %         while ln<=lane_width_size
       %             right_width_sum=right_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
       %             ln=ln+1;
       %         end
       %     else
       %         ln=ceil(mid_lane);
       %         right_width_sum=scenario.RoadSpecifications.Lanes.Width(1,ln)/2.0;
       %         ln=ln+1;
       %         while ln>0
       %             right_width_sum=right_width_sum+ scenario.RoadSpecifications.Lanes.Width(1,ln);
       %             ln=ln+1;
       %         end
       %     end
       %     right_boundary=mid_road(1,1) + right_width_sum;
       %
%
       %      %sample a point between boundary at the start of road
       %     x_val=left_boundary + rand*(right_boundary-left_boundary);
%
       %     next_waypoint= [left_boundary scenario.RoadSpecifications.Centers(i,2) scenario.RoadSpecifications.Centers(i,3)];
       %     waypoints_total=[waypoints_total; next_waypoint];
       %   end
       %   %all_waypoints=veh_obj.waypoints;
%
       %end
%
       %function [min_val, nearest_index] = find_nearest_point(veh_obj,point, array)
       %     % Calculate the squared Euclidean distances between the given point and each element of the array
       %     distances = sum((array - point).^2, 2);
       % 
       %     % Find the index of the nearest point
       %     [min_val, nearest_index] = min(distances);
       %end
%
       %function [dist_of_next_from_border1, dist_of_next_from_border2, flag, flag_next_border1, flag_next_border2,flag_current_border1,flag_current_border2]=check_if_point_in_trapezium(veh_obj, border1, border2, point_x, point_y, prev_x, prev_y)
       %     flag=false;
       %     [dist_from_border1,nearest_index_border1] = find_nearest_point(veh_obj,[point_x, point_y], border1(:,1:2));
       %     [dist_from_border2, nearest_index_border2] = find_nearest_point(veh_obj,[point_x, point_y], border2(:,1:2));
       %     if dist_from_border1<dist_from_border2
       %         nearest_index_border=nearest_index_border1;
       %     else
       %         nearest_index_border=nearest_index_border2;
       %     end
%
       %     if nearest_index_border-5>0
       %         border1_left_index=nearest_index_border-5;
       %         border2_left_index=nearest_index_border-5;
       %     else
       %         border1_left_index=1;
       %         border2_left_index=1;
       %     end
%
       %     
       %         
       %     
%
       %     if nearest_index_border+5<=length(border1)
       %         border1_right_index=nearest_index_border+5;
       %         border2_right_index=nearest_index_border+5;
       %     else
       %         border1_right_index=length(border1);
       %         border2_right_index=length(border2);
       %     end
%
       %     
%
%
       %     %if nearest_index_border1-5>0
       %     %    border1_left_index=nearest_index_border1-5;
       %     %else
       %     %    border1_left_index=1;
       %     %end
%%
       %     %if nearest_index_border2-5>0
       %     %    border2_left_index=nearest_index_border2-5;
       %     %else
       %     %    border2_left_index=1;
       %     %end
%%
       %     %if nearest_index_border1+5<=length(border1)
       %     %    border1_right_index=nearest_index_border1+5;
       %     %else
       %     %    border1_right_index=length(border1);
       %     %end
%%
       %     %if nearest_index_border2+5<=length(border2)
       %     %    border2_right_index=nearest_index_border2+5;
       %     %else
       %     %    border2_right_index=length(border2);
       %     %end
%
       %     if border1(border1_left_index,1)>border2(border2_left_index,1)
       %         start_x=border2(border2_left_index,1);
       %     else
       %         start_x=border1(border1_left_index,1);
       %     end
%
       %     if border1(border1_right_index,1)>border2(border2_right_index,1)
       %         end_x=border2(border2_right_index,1);
       %     else
       %         end_x=border1(border1_right_index,1);
       %     end
%
       %     if border1(border1_left_index,2)>border2(border2_left_index,2)
       %         start_y=border2(border2_left_index,2);
       %     else
       %         start_y=border1(border1_left_index,2);
       %     end
%
       %     if border1(border1_right_index,2)>border2(border2_right_index,2)
       %         end_y=border2(border2_right_index,2);
       %     else
       %         end_y=border1(border1_right_index,2);
       %     end
%
       %     if start_x<end_x
       %         left_x=start_x;
       %         right_x=end_x;
       %     else
       %         left_x=end_x;
       %         right_x=start_x;
       %     end
%
       %     if start_y<end_y
       %         left_y=start_y;
       %         right_y=end_y;
       %     else
       %         left_y=end_y;
       %         right_y=start_y;
       %     end
%
%
       %     %if ((point_x<=end_x && point_x>=start_x)||(point_x>=end_x && point_x<=start_x)) && ((point_y<=end_y && point_y>=start_y)||(point_y>=end_y && point_y<=start_y))
       %      if ((point_x<=right_x+0.5 && point_x>=left_x-0.5)) && ((point_y<=right_y+0.5 && point_y>=left_y-0.5))
       %         
       %         flag=true;
       %     end
%
       %     slope_border_1=(border1(border1_right_index,2)-border1(border1_left_index,2))/(border1(border1_right_index,1)-border1(border1_left_index,1)) ; %(y2-y1)/(x2-x1)
       %     intercept_border_1= border1(border1_right_index,2)-slope_border_1*border1(border1_right_index,1); %(y-mx)
       %     slope_border_2=(border2(border2_right_index,2)-border2(border2_left_index,2))/(border2(border2_right_index,1)-border2(border2_left_index,1)) ; %(y2-y1)/(x2-x1)
       %     intercept_border_2= border2(border2_right_index,2)-slope_border_2*border2(border2_right_index,1); %(y-mx)
       %     dist_of_next_from_border1=abs(point_y-slope_border_1*point_x-intercept_border_1) / sqrt(1*1+(slope_border_1)*(slope_border_1));
       %     dist_of_next_from_border2=abs(point_y-slope_border_2*point_x-intercept_border_2) / sqrt(1*1+(slope_border_2)*(slope_border_2));
%
       %     curr_value_point=prev_y-slope_border_1*prev_x-intercept_border_1; %y-mx-c to decide sign
       %     if curr_value_point<0
       %         flag_current_border1=-1;
       %     else
       %         flag_current_border1=1;
       %     end
       %     
       %     curr_value_point=prev_y-slope_border_2*prev_x-intercept_border_2; %y-mx-c to decide sign
       %     if curr_value_point<0
       %         flag_current_border2=-1;
       %     else
       %         flag_current_border2=1;
       %     end
       %     
       %     next_value_point=point_y-slope_border_1*point_x-intercept_border_1; %y-mx-c to decide sign
       %     if next_value_point<0
       %         flag_next_border1=-1;
       %     else
       %         flag_next_border1=1;
       %     end
       %     next_value_point=point_y-slope_border_2*point_x-intercept_border_2; %y-mx-c to decide sign
       %     if next_value_point<0
       %         flag_next_border2=-1;
       %     else
       %         flag_next_border2=1;
       %     end
%
%
       %end
        
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

            for current_road_block=1:current_road_blocks
            
                    left_bottom_x=border_1(current_road_block,1);
                    left_bottom_y=border_1(current_road_block,2);
                    left_top_x   =border_2(current_road_block,1);
                    left_top_y   =border_2(current_road_block,2);
                    right_bottom_x=border_1(current_road_block,1);
                    right_bottom_y=border_1(current_road_block,2);
                    right_top_x   =border_2(current_road_block,1);
                    right_top_y   =border_2(current_road_block,2);
                    
                    %if current_road_block-2>=1
                    %    if current_road_block+2<=length(border_1)
                    %        left_bottom_x=border_1(current_road_block-2,1);
                    %        left_bottom_y=border_1(current_road_block-2,2);
                    %        left_top_x   =border_2(current_road_block-2,1);
                    %        left_top_y   =border_2(current_road_block-2,2);
                    %        right_bottom_x=border_1(current_road_block+2,1);
                    %        right_bottom_y=border_1(current_road_block+2,2);
                    %        right_top_x   =border_2(current_road_block+2,1);
                    %        right_top_y   =border_2(current_road_block+2,2);
                    %    else
                    %        left_bottom_x=border_1(current_road_block-2,1);
                    %        left_bottom_y=border_1(current_road_block-2,2);
                    %        left_top_x   =border_2(current_road_block-2,1);
                    %        left_top_y   =border_2(current_road_block-2,2);
                    %        right_bottom_x=border_1(current_road_block,1);
                    %        right_bottom_y=border_1(current_road_block,2);
                    %        right_top_x   =border_2(current_road_block,1);
                    %        right_top_y   =border_2(current_road_block,2);
                    %    end
%       
                    %else
                    %    if current_road_block +2 <=length(border_1)
                    %        left_bottom_x=border_1(current_road_block,1);
                    %        left_bottom_y=border_1(current_road_block,2);
                    %        left_top_x   =border_2(current_road_block,1);
                    %        left_top_y   =border_2(current_road_block,2);
                    %        right_bottom_x=border_1(current_road_block+2,1);
                    %        right_bottom_y=border_1(current_road_block+2,2);
                    %        right_top_x   =border_2(current_road_block+2,1);
                    %        right_top_y   =border_2(current_road_block+2,2);
                    %    else
                    %        left_bottom_x=border_1(current_road_block,1);
                    %        left_bottom_y=border_1(current_road_block,2);
                    %        left_top_x   =border_2(current_road_block,1);
                    %        left_top_y   =border_2(current_road_block,2);
                    %        right_bottom_x=border_1(current_road_block,1);
                    %        right_bottom_y=border_1(current_road_block,2);
                    %        right_top_x   =border_2(current_road_block,1);
                    %        right_top_y   =border_2(current_road_block,2);
                    %    end
                    %end
        
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

       function waypoints_total=bicycle_model_with_road_constraints_3(veh_obj, scenario)
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
            
           dist_from_block_end=0.6;
           dist_of_new_node=3.0;
           current_road_block=1;
           tree=dictionary;
           valid_nodes={[-156.0 -28.2]};
           while current_road_block<length(border_1)-2
               disp(current_road_block);
               %sample a
               sampled_x=-180+ ((-40)-(-180))*rand; %((-300)+rand*(300-(-300)));
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
                                %index = find([valid_nodes{:}] == [target_x target_y]);
                                if index == 0
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
                                
                                        scatter(border_1(:,1), border_1(:,2));
                                        hold on 
                                        scatter(border_2(:,1), border_2(:,2));
                                        hold on
                                        scatter(node_x, node_y);
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



       function waypoints_total=bicycle_model_with_road_constraints_2(veh_obj, scenario)
           centers = scenario.RoadSpecifications.Centers;
           new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
           laneSpecification = lanespec([1 1]);
           road(new_scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
           rbScenario = roadBoundaries(new_scenario_obj);

           %differentiate the boundaries individually
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
    
           %initialize first state
           num_waypoints=length(border_1);
           state=[-156.7 -28.2 -pi/1.5 0];%[border_1(1,1)-1 border_1(1,2)-1 -pi/2.5 0];%[border_1(1,1)-1 border_1(1,2)-1 pi/2 -pi/36];
           L=1;
           delta_t=0.05;
           prev_input_vel=10;%(5 + rand*(50-5)) ;
           prev_input_steer_angle_rate=0;%-pi/36;%(-pi/4 + rand*(pi/4-(-pi/4)));
           road_block_iter=2;
           slack_distance=0.5;

           for i = 2: num_waypoints
               disp(i);
                                                                               
                while(1)
                    if i==2
                        input_vel=prev_input_vel ;
                        input_steer_angle_rate=prev_input_steer_angle_rate;
                    else
                        input_vel=prev_input_vel + ((-5 + rand*(5-(-5))));
                        input_steer_angle_rate=((-0.5 + rand*(0.5-(-0.5))));%prev_input_steer_angle_rate;% + ((-0.001 + rand*(0.001-(-0.001))));%((-pi/36 + rand*(pi/36-(-pi/36))));
                    end
    
                    %considering rear wheel axle
                    %x_dot= input_vel*cos(state(i-1,3));
                    %y_dot= input_vel*sin(state(i-1,3));
                    %%rear wheel
                    %R= L/tan(state(i-1,4));
                    %omega=input_vel/R;
                    %theta_dot=(input_vel*tan(state(i-1,4)))/L;
                    %steer_dot=input_steer_angle_rate;
    
                    %considering front wheel axle
                    x_dot= input_vel*cos(state(i-1,3)+state(i-1,4));
                    y_dot= input_vel*sin(state(i-1,3)+state(i-1,4));
                    %front wheel
                    R=L/sin(state(i-1,4));
                    omega=input_vel/R;
                    theta_dot=(input_vel*sin(state(i-1,4)))/L;
                    steer_dot=input_steer_angle_rate;
                    
                    
                    x_next=state(i-1,1) + x_dot*delta_t;
                    y_next=state(i-1,2) + y_dot*delta_t;
                    theta_next=state(i-1,3) + theta_dot*delta_t;
                    steer_next=state(i-1,4) + steer_dot*delta_t;
                    next_state=[x_next y_next theta_next steer_next];

                    if i>2

                        if (((next_state(1)>=border_1(road_block_iter,1)-slack_distance && next_state(1)<=border_2(road_block_iter,1)+slack_distance) ||...
                           (next_state(1)<=border_1(road_block_iter,1) +slack_distance && next_state(1)>=border_2(road_block_iter,1)-slack_distance)) && ...
                           ((next_state(1)>=border_1(road_block_iter+2,1)-slack_distance && next_state(1)<=border_2(road_block_iter+2,1)+slack_distance) ||...
                           (next_state(1)<=border_1(road_block_iter+2,1)+slack_distance && next_state(1)>=border_2(road_block_iter+2,1)-slack_distance))) && ...
                           (((next_state(2)>=border_1(road_block_iter,2)-slack_distance && next_state(2)<=border_1(road_block_iter+2,2)+slack_distance) ||...
                           (next_state(2)<=border_1(road_block_iter,2)+slack_distance && next_state(2)>=border_1(road_block_iter+2,2)-slack_distance)) && ...
                           ((next_state(2)>=border_2(road_block_iter,2)-slack_distance && next_state(2)<=border_2(road_block_iter+2,2)+slack_distance) ||...
                           (next_state(2)<=border_2(road_block_iter,2)+slack_distance && next_state(2)>=border_2(road_block_iter+2,2)-slack_distance))) && ...
                           road_block_iter+2<length(border_2)


                                if (next_state(2)>=border_1(road_block_iter+1,2)) || (next_state(2)>=border_2(road_block_iter+1,2))
                                    road_block_iter = road_block_iter+1;
                                end
                                break


                        %else 
                         %  break
                        end

                    else
                        break
                    end
                end
    
                    state=[state; next_state];
               

           end
           waypoints_total=zeros(length(state),3);
           waypoints_total(:,1:2)=state(:,1:2);
           
       end

       %function waypoints_total=bicycle_model_with_road_constraints(veh_obj, scenario)
       %     centers = scenario.RoadSpecifications.Centers;
       %     %yyStart = centers(1,2);
       %     %yyEnd = centers(end,2);
       %     %newYs = linspace(yyStart,yyEnd,50);
       %     %newXs = interp1(centers(:,2),centers(:,1),newYs,'pchip');
       %     %newZs = interp1(centers(:,2),centers(:,3),newYs,'pchip');
       %     %newCenters = [newXs' newYs' newZs'];
       %     
       %     %get road boundaries
       %     new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
       %     laneSpecification = lanespec([1 1]);
       %     road(new_scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
       %     rbScenario = roadBoundaries(new_scenario_obj);
       %     
       %     %differentiate the boundaries individually
       %     border_total=rbScenario{1,1};
       %     if mod(border_total,2)==0
       %         first_border_end=floor(length(border_total)/2);
       %         second_border_start=first_border_end+1;
       %     else
       %         first_border_end=floor(length(border_total)/2);
       %         second_border_start=first_border_end+2;
       %     end
       %     border_1=border_total(1:first_border_end, :);
       %     border_2=flip(border_total(second_border_start:length(border_total), :));
       %     
       %     %initialize first state
       %     num_waypoints=length(border_1);
       %     state=[border_1(1,1)-1 border_1(1,2)-1 -pi/2.5 0];%[border_1(1,1)-1 border_1(1,2)-1 pi/2 -pi/36];
       %     L=1;
       %     delta_t=0.05;
       %     prev_input_vel=10;%(5 + rand*(50-5)) ;
       %     prev_input_steer_angle_rate=0;%-pi/36;%(-pi/4 + rand*(pi/4-(-pi/4)));
       %     next_trapezium_pt=1;
       %     flag_set_manual_steering=false;
       %     %flag_in_road=false;
       %     last_inverted_frame=0;
       %     last_offroad_inverted_frame=0;
       %     %kinematic bicycle model to estimate next states
       %     for i = 2: num_waypoints
       %        disp(i);
       %        %loop=0;
       %        %while 1
       %         flag_current_border1=0; % to check which size of the border line, the point lies
       %         flag_current_border2=0;
       %         flag_next_border1=0; % decide flag = 1 if sign positive, else flag = -1
       %         flag_next_border2=0;
       %                      
       %                                                 
       %         %while(1)
       %         if i==2
       %             input_vel=prev_input_vel ;
       %             input_steer_angle_rate=prev_input_steer_angle_rate;
       %         else
       %             input_vel=prev_input_vel + ((-5 + rand*(5-(-5))));
       %             if flag_set_manual_steering == false
       %                 %prev_input_steer_angle_rate=
       %                 input_steer_angle_rate=((-0.001 + rand*(0.001-(-0.001))));%prev_input_steer_angle_rate;% + ((-0.001 + rand*(0.001-(-0.001))));%((-pi/36 + rand*(pi/36-(-pi/36))));
       %             else
       %                 input_steer_angle_rate=prev_input_steer_angle_rate;%0;prev_input_steer_angle_rate;
       %                 %flag_set_manual_steering=false;
       %                 prev_input_steer_angle_rate=0;
       %             end
       %         end
%
       %         %considering rear wheel axle
       %         %x_dot= input_vel*cos(state(i-1,3));
       %         %y_dot= input_vel*sin(state(i-1,3));
       %         %%rear wheel
       %         %R= L/tan(state(i-1,4));
       %         %omega=input_vel/R;
       %         %theta_dot=(input_vel*tan(state(i-1,4)))/L;
       %         %steer_dot=input_steer_angle_rate;
%
       %         %considering front wheel axle
       %         x_dot= input_vel*cos(state(i-1,3)+state(i-1,4));
       %         y_dot= input_vel*sin(state(i-1,3)+state(i-1,4));
       %         %front wheel
       %         R=L/sin(state(i-1,4));
       %         omega=input_vel/R;
       %         theta_dot=(input_vel*sin(state(i-1,4)))/L;
       %         steer_dot=input_steer_angle_rate;
       %         
       %         
       %         x_next=state(i-1,1) + x_dot*delta_t;
       %         y_next=state(i-1,2) + y_dot*delta_t;
       %         theta_next=state(i-1,3) + theta_dot*delta_t;
       %         steer_next=state(i-1,4) + steer_dot*delta_t;
       %         next_state=[x_next y_next theta_next steer_next];
%
       %         
       %         
       %         
       %        
       %         [dist_of_next_from_border1, dist_of_next_from_border2, flag_point_in_trapezium,flag_next_border1, flag_next_border2,flag_current_border1,flag_current_border2]=check_if_point_in_trapezium(veh_obj, border_1, border_2,  x_next, y_next, state(i-1,1), state(i-1,2));
       %         if (flag_current_border1==flag_next_border1)&&(flag_current_border2==flag_next_border2) && ((flag_point_in_trapezium==true) ||i<3)
       %             %disp("Inside Trapezium");
       %             prev_input_vel=input_vel;
       %             %prev_input_steer_angle_rate=input_steer_angle_rate;
       %             %break;
       %             flag_set_manual_steering=false;
       %             %flag_in_road=true;
       %             if (dist_of_next_from_border2<0.8 && (last_inverted_frame==0 || last_inverted_frame-i>3) ) %&& i>4
       %               %if input_steer_angle_rate<-0
       %                   %prev_input_steer_angle_rate=(-1)*prev_input_steer_angle_rate;%pi;%input_steer_angle_rate + pi;
       %                   prev_input_steer_angle_rate=(-1)*state(i-1,4)*10 ; %-pi/6;
       %                   flag_set_manual_steering=true;
       %                   last_inverted_frame=i;
       %                   
       %             
       %             elseif  (dist_of_next_from_border1<0.8 && (last_inverted_frame==0 || last_inverted_frame-i>3))
       %                  prev_input_steer_angle_rate=(-1)*state(i-1,4)*10 ; %pi/6;
       %                  flag_set_manual_steering=true;
       %                   last_inverted_frame=i;
       %             else
       %                 prev_input_steer_angle_rate=input_steer_angle_rate;
%
       %             end
       %                 
       %         else
       %             
       %             %if flag_in_road== true && i-last_inverted_frame>5
       %                 %prev_input_steer_angle_rate=(-1)*prev_input_steer_angle_rate;
       %                 
%
       %             %end
       %             
       %             if (dist_of_next_from_border2<1.5 && (last_offroad_inverted_frame==0 || last_offroad_inverted_frame-i>3) ) %&& i>4
       %               %if input_steer_angle_rate<-0
       %                   %prev_input_steer_angle_rate=(-1)*prev_input_steer_angle_rate;%pi;%input_steer_angle_rate + pi;
       %                   prev_input_steer_angle_rate= -pi/6;
       %                   flag_set_manual_steering=true;
       %                   last_offroad_inverted_frame=i;
       %                   disp("Off Road Close");
       %             
       %             elseif  (dist_of_next_from_border1<1.5 && (last_offroad_inverted_frame==0 && last_offroad_inverted_frame-i>3))
       %                  prev_input_steer_angle_rate=pi/6;
       %                  flag_set_manual_steering=true;
       %                  last_offroad_inverted_frame=i;
       %                  disp("Off Road Close");
       %             else
       %                 prev_input_steer_angle_rate=0;%-pi/6;
       %                 flag_set_manual_steering=true;
       %                 disp("Off Road");
       %                 %flag_in_road=false;
%
       %             end
       %             %flag_set_manual_steering=true;
       %             %last_inverted_frame=i;
%
       %             %if input_steer_angle_rate<0
       %             %      input_steer_angle_rate=input_steer_angle_rate+pi;%input_steer_angle_rate + pi;
       %             %      input_vel=input_vel+10;
       %             %  else
       %             %      input_steer_angle_rate=input_steer_angle_rate-pi;%input_steer_angle_rate - pi;
       %             %      input_vel=input_vel+10;
       %             %  end
       %             %break;
       %         %    loop=loop+1;
       %         %    
       %         %
       %         end
%
       %         %end
%
       %        %end
%
       %         
%
       %      state=[state; next_state];
%
%
       %     end
       % 
%
%
       %     %compute straight line between two road boundaries and find if
       %     %the computed point lies around this line and not going offroad
%
       %     waypoints_total=zeros(length(state),3);
       %     waypoints_total(:,1:2)=state(:,1:2);
       %end

       function veh_obj=vehicle_movement(veh_obj, scenario)%iteration, waypoints, speed)

           %rbScenario = roadBoundaries();
           new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
           laneSpecification = lanespec([1 1]);
           road(new_scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
           rbScenario = roadBoundaries(new_scenario_obj);
           %rbEgo2 = driving.scenario.roadBoundariesToEgo(rbScenario,ego);
           %bep = birdsEyePlot;
           %lbp = laneBoundaryPlotter(bep,'DisplayName','Road boundaries');
           %plotLaneBoundary(lbp,{rbEgo2})

           %waypoints_total=kinematic_bicycle_model(veh_obj, scenario);
           %waypoints_total=generate_waypoints(veh_obj, scenario);
           waypoints_total=bicycle_model_with_road_constraints_3(veh_obj, scenario);
           %waypoints_total = [-161.9 -28.2 0;
           %             -161.4 -28.4 0;
           %             -160.4 -38.7 0;
           %             -159.7 -43.8 0;
           %             -157 -46.3 0;
           %             -154.7 -49.5 0;
           %             -154.2 -53.5 0;
           %             -152.8 -58.4 0;
           %             -152.6 -61.8 0];
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