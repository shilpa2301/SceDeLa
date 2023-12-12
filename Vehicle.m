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

            if(length(S.x3)==1 ||(dist_func(veh_obj,S.x3(1),S.y3(1), x2, y2) <= dist_func(veh_obj,S.x3(2),S.y3(2), x2, y2) ))
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

            %start_block= 1;
            %end_block =start_block+block_threshold;
            %for bt = 2:-1:1
            %    if nearest_node_block_id - bt>=1
            %        start_block = nearest_node_block_id - bt;
            %        break
            %    end
            %end
%
            %for bt = block_threshold:-1:1
            %    if nearest_node_block_id + bt<=length(border_1)
            %        end_block = nearest_node_block_id + bt;
            %        break
            %    end
            %end
           %

            start_block = nearest_node_block_id;
       
            end_block = nearest_node_block_id+1;
            for current_road_block=start_block:end_block
            
                    left_bottom_x=border_1(current_road_block,1);
                    left_bottom_y=border_1(current_road_block,2);
                    left_top_x   =border_2(current_road_block,1);
                    left_top_y   =border_2(current_road_block,2);
                    %if current_road_block+block_threshold<=length(border_1)
                    %    end_check = current_road_block+block_threshold;
                    %else
                    %    end_check = length(border_1);
                    %end
                    %right_bottom_x=border_1(end_check,1);
                    %right_bottom_y=border_1(end_check,2);
                    %right_top_x   =border_2(end_check,1);
                    %right_top_y   =border_2(end_check,2);

                    if current_road_block+1 <=length(border_1)

                        right_bottom_x=border_1(current_road_block+1,1);
                        right_bottom_y=border_1(current_road_block+1,2);
                        right_top_x   =border_2(current_road_block+1,1);
                        right_top_y   =border_2(current_road_block+1,2);
                        
                    
            
                        %distance_from_block_end=-1;
                        corresp_road_block = -1;
            
                        if inpolygon(target_x,target_y,[left_bottom_x right_bottom_x right_top_x left_top_x], ...
                                [left_bottom_y right_bottom_y right_top_y left_top_y])
                            validity_flag=1;
            
                            %calculate distance from block end
                            pt=[target_x target_y];
                            v1=[right_bottom_x right_bottom_y];
                            v2=[right_top_x right_top_y];
                            %if current_road_block== start_block
                            corresp_road_block = current_road_block;
                            %else
                             %   corresp_road_block = end_check;
                            %end
                            break
            
                        end
                    end
            end
%
       
       end
          
            function [validity_flag, corresp_road_block]=find_if_target_node_valid(veh_obj, target_x, target_y, border_1, border_2,block_threshold, nearest_node_block_id)

            validity_flag=false;
            corresp_road_block = -1;
            
            [validity_flag,corresp_road_block ]=find_if_node_inside_any_road_block(veh_obj, target_x, target_y, border_1, border_2,block_threshold, nearest_node_block_id);
            

            end


            function [sampled_x, sampled_y] = find_temporary_goal(veh_obj, random_block_id, border_1, border_2)
                b1_first = border_1(random_block_id,:);
                b1_next = border_1(random_block_id+1,:);
                b2_first = border_2(random_block_id,:);
                b2_next = border_2(random_block_id+1,:);

                all_x = [b1_first(1) b2_first(1) b1_next(1) b2_next(1)];
                all_y = [b1_first(2) b2_first(2) b1_next(2) b2_next(2)];
                [min_x, max_x] = bounds(all_x);
                [min_y, max_y] = bounds(all_y);

                rng('shuffle')
                % sampled_x=normrnd((min_x+max_x)/2.0, (max_x-min_x)/2.0);%((-300)+rand*(300-(-300)));
                % sampled_y=normrnd((min_y+max_y)/2.0, (max_y-min_y)/2.0);
                % 

                
                heading = atan2((b1_next(2)-b1_first(2)),(b1_next(1)-b1_first(1)));
                % rng('shuffle');
                rng('shuffle')
                sampled_x=(min_x)+(max_x-min_x)*rand;
                rng('shuffle')
                sampled_y=(min_y)+(max_y-min_y)*rand;
                % if heading < -1.31 
                %      rng('shuffle')
                %     sampled_x=normrnd((min_x+max_x)/2.0, (max_x-min_x+40));%((-300)+rand*(300-(-300)));
                %     sampled_y=normrnd((min_y+max_y)/2.0, (max_y-min_y+10));
                %     sampled_x=(min_x)+(max_x-min_x)*rand;
                %     rng('shuffle')
                %      sampled_y=(min_y)+(max_y-min_y)*rand;
                % 
                % elseif heading <0.53
                %      rng('shuffle')
                %     sampled_x=normrnd((min_x+max_x)/2.0, (max_x-min_x+10));%((-300)+rand*(300-(-300)));
                %     sampled_y=normrnd((min_y+max_y)/2.0, (max_y-min_y+10));
                % else
                %      rng('shuffle')
                %      sampled_x=(min_x)+(max_x-min_x)*rand;
                %      rng('shuffle')
                %      sampled_y=(min_y)+(max_y-min_y)*rand;
                %     sampled_x=normrnd((min_x+max_x)/2.0, (max_x-min_x+10));
                %     sampled_y=normrnd((min_y+max_y)/2.0, (max_y-min_y+20));%-180 + ((-20)-(-180))*rand; %((-300)+rand*(300-(-300)); 
                % end
                % sampled_x=min_x-20.0+ ((max_x+20.0)-(min_x-20.0))*rand; %((-300)+rand*(300-(-300)));
                % sampled_y=min_y + ((max_y)-(min_y))*rand;%-180 + ((-20)-(-180))*rand; %((-300)+rand*(300-(-300)); 
                % 
                scatter(border_1(:,1), border_1(:,2),5, 'red', "filled");
                hold on 
                scatter(border_2(:,1), border_2(:,2),5, 'red', "filled");
                hold on
                scatter(sampled_x, sampled_y, 'green', "filled");
                hold on                
                rectangle('Position', [min_x, min_y, max_x-min_x, max_y-min_y],...
                  'EdgeColor','b', 'LineWidth', 3)
                close 
            end

            function lane_id = find_lane(veh_obj, considered_x, considered_y, corresp_road_block, border_1, border_2, centre_line)
                lane_id =0;
                
                border_1_first = border_1(corresp_road_block,:);
                border_1_next = border_1(corresp_road_block+1,:);
                border_2_first = border_2(corresp_road_block,:);
                border_2_next = border_2(corresp_road_block+1,:);
                centre_first = centre_line(corresp_road_block,:);
                centre_next = centre_line(corresp_road_block+1,:);

                 
                if inpolygon(considered_x,considered_y,[border_1_first(1) border_1_next(1) centre_next(1) centre_first(1)], ...
                                [border_1_first(2) border_1_next(2) centre_next(2) centre_first(2)])
                    lane_id =1;
                else
                    lane_id = 2;
                end
            end

       function [tree, valid_nodes]=sampled_scenario(veh_obj, scenario, veh_id, num_paths)

           
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
           
           new_border_1=[];
           new_border_2=[];
           for bd = 1:length(border_1)
                if mod(bd,2) == 0
                    new_border_1=[new_border_1; border_1(bd,:)];
                    new_border_2=[new_border_2; border_2(bd,:)];
                end
           end
           border_1=new_border_1;
           border_2=new_border_2;
           centre_line=(new_border_1 + new_border_2)/2.0;
           
           %multi_Agent

           %inputs=1:num_paths;
           % results = cell(1, num_paths);

           
                % if veh_id ==1
                %  centre_line=(new_border_1 + new_border_2)/2.0;
                %  outer_border=border_2;
                %  border_2 = centre_line;
                % else
                %  centre_line=(new_border_1 + new_border_2)/2.0;
                %  outer_border=border_1;
                %  border_1 = centre_line;
                % 
                % end
                
                heading_diff=0;
                second = border_1(2,:);
                first = border_1(1,:);       
                prev=(atan2((second(2)-first(2)),(second(1)-first(1)))); 
                for ii=2:length(border_1)-1
                    second = border_1(ii+1,:);
                    first = border_1(ii,:);       
                    curr=(atan2((second(2)-first(2)),(second(1)-first(1))));
                    heading_diff=[heading_diff;(curr-prev)];
                    disp(curr-prev);
                    prev=curr;
        
                end
        
                %dist_from_block_end=1.0;
               
                %multi_Agent
                border_1_init = border_1(1,:);
                border_2_init = border_2(1,:);
                round=0;
                while(1)
                    round=round+1;

                    for p = 2:num_paths -1
        
                         dist_of_new_node=3.0;%3.0; %3.0;
                         %current_road_block=1;
                         %tree=dictionary;
                         tree = [];
                    
                         second = border_1(2,:);
                         first = border_1(1,:);
                         heading_last_road = atan2((second(2)-first(2)),(second(1)-first(1)));
            
                         flag_distant_from_other_points = 0;
                         threshold_dist_from_other_points = 0.5;%1.0;%1.0;%1.0;
                         block_threshold = 1;%6; %with dist = 1.0, this val = 6
                         knn=40;
                         velocity_change_threshold = 5.4;%20km/hr 2.7; % 10km/hr per frame
                         steer_angle_rate_change_threshold = 0;%0.17;%0.1;%5 degree, prev = 1.0; % 0.5 radian = 30 degrees % 0.17; %10 degrees in radians
                         init_velocity = 5.6;%16.8;%5.6;%40km/hr prev=0.2;% working 1.0; % 100 km/h = 27.8 m/s; 12.5; %45 maikm/hr
                         init_steer_angle_rate = 0.0; % driving straight
                         L=1.5;%mts, prev = 0.5;
                         delta_t=0.5;%0.01;%in secs, prev=3.0;
        
        
                        x_init = border_1_init(1) +p*( border_2_init(1) - border_1_init(1))/num_paths ;
                        y_init = border_1_init(2) -p*( border_1_init(2) -border_2_init(2))/num_paths ;
                        disp("p="+ string(p)+", x,y="+string(x_init)+","+string(y_init));
                        rng('shuffle')
                        head_err = -0.17+ ((0.17)-(-0.17))*rand;
                        if p<=5
                            lane_id=1;
                        else
                            lane_id=2;
                        end
                        valid_nodes={[x_init y_init-0.1 heading_last_road+head_err 0.01 1 lane_id]};%{[-157 -31.5 1.00 0.01 1]};
                        % if veh_id ==1
                        %  valid_nodes={[-162.3 -29 heading_last_road 0.01 1]};%{[-160 -28.2 1.05 0.01 1]}; % [x y heading steering_angle blockid laneid] and -pi/5 = -0.63 = 36 degrees % 60 degrees = 1.05
                        % else
                        %  valid_nodes={[-157 -31.5 heading_last_road 0.01 1]};%{[-157 -31.5 1.00 0.01 1]};
                        % end
                    
                    
                        corresp_road_block = 1;
                        frame = 0;
                        last_road_block = 1;
                    
                        num_goal=0;
                        while num_goal <=10
                        %while last_road_block<length(border_1)-block_threshold
                            %prev_heading = heading_last_road;
                             frame=frame+1;
                             disp("p="+ string(p)+", frame=" + string(frame)+", last block="+last_road_block);
                            %disp(current_road_block);
                            %sample a
                    
                             % sampled_x=normrnd(-150, 100);%-240+ ((-40)-(-240))*rand; %((-300)+rand*(300-(-300)));
                             % sampled_y=normrnd(-70, 50);%-90 + ((-20)-(-90))*rand; %((-300)+rand*(300-(-300));
                             % width_of_search = length(border_1) - 1-last_road_block;
                    
                             if length(border_1) -1 - last_road_block >=5
                                  rng('shuffle')
                                random_block_id = randi(5);
                             else
                                 random_block_id = length(border_1) - 1 - last_road_block;
                             end
                             random_block_id = last_road_block+random_block_id ;
                             rng('shuffle');
                             % random_block_id = randi(length(border_1)-1);
                             % disp('block temp goal='+string(random_block_id));
                             [sampled_x, sampled_y] = find_temporary_goal(veh_obj, random_block_id, border_1, border_2);
        
                    
                    
                    
                            %Find if the point's distance id < delta_distance=1.0
                                     [nearest_node_list, dist_list] = find_k_nearest_node_in_tree(veh_obj, valid_nodes, sampled_x, sampled_y,knn);
                                     flag_find_new_node =0;
                                     %[nearest_node_id, dist_from_curr_state]=find_nearest_node_in_tree(veh_obj, valid_nodes, sampled_x, sampled_y);
                                     for nn = 1:length(nearest_node_list)
                                     %dist_from_curr_state=dist_func(veh_obj, state(end,1), state(end,2), sampled_x, sampled_y);
                                         nearest_node_id = nearest_node_list(nn);
                                         dist_from_curr_state = dist_list(nn);
                                         % disp(["checking with nearest neighbor",num2str(nn)])
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
                    
                                                 %if last_road_block >1
                                                 % last = border_1(last_road_block+1,:);
                                                 % prev = border_1(last_road_block,:);
                                                 % heading_last_road = atan2((last(2)-prev(2)),(last(1)-prev(1)));
                                                 % diff=heading_last_road - prev_heading;
                                                 % if diff ~=0
                                                 %     sampled_steering_angle_rate_changed = diff;
                                                 % else
                                                 %     sampled_steering_angle_rate_changed = 0.17;
                                                 % end
                                                 % prev_heading = heading_last_road;
                                                 %end
                    
                                                 prev_heading = start_node{1}(3);
                                                 blockid = start_node{1}(5);
                                                 block_heading = atan2((border_1(blockid+1,2)- border_1(blockid,2)),(border_1(blockid+1,1)- border_1(blockid,1)));
                    
                    
                                                 %sampled points and bicycle model
                                                 sampled_steering_angle_rate_changed = zeros(1,10);
                                                 sampled_velocity_change = zeros(1,10);
                    
                    
                    
                                                      % sampled_steering_angle_rate_changed =normrnd(steer_angle_rate_change_threshold, 0.5, [1,10]);
                                                     %if mod(frame, 3) ==0
                                                        if abs(heading_diff(start_node{1}(5)))>0.1%0.01
                                                        %if abs(start_node{1}(5)) > 0.01
                                                            % sampled_steering_angle_rate_changed =(-0.9) +(0.9-(-0.9))*rand(1,10);
                                                             rng('shuffle')
                                                            sampled_steering_angle_rate_changed =normrnd(steer_angle_rate_change_threshold, 0.5, [1,10]);
                                                            velocity_change_threshold = normrnd(0, 1.0);
                                                         elseif abs(heading_diff(start_node{1}(5)))>0.03%0.01
                                                        % 
                                                             rng('shuffle')
                                                             sampled_steering_angle_rate_changed =normrnd(steer_angle_rate_change_threshold, 0.5, [1,10]);
                                                             velocity_change_threshold = normrnd(0, 1.7);
                                                        % elseif abs(heading_diff(start_node{1}(5)))>0.01%0.01
                                                        % %if abs(start_node{1}(5)) > 0.01
                                                            % sampled_steering_angle_rate_changed =(-0.75) +(0.75-(-0.75))*rand(1,10);
                                                            % sampled_steering_angle_rate_changed =normrnd(steer_angle_rate_change_threshold, 0.6, [1,10]);
                                                        else
                                                            % sampled_steering_angle_rate_changed =(-0.5) +(0.5-(-0.5))*rand(1,10);
                                                             rng('shuffle')
                                                            sampled_steering_angle_rate_changed =normrnd(steer_angle_rate_change_threshold, 0.2, [1,10]);
                                                            velocity_change_threshold = normrnd(0, 2.7);
                    
                                                        end
        
                    
                                                 %sampled_steering_angle_rate_changed = normrnd(steer_angle_rate_change_threshold, 0.001, [1,10]);
                    
                                                 sampled_velocity_change=normrnd(velocity_change_threshold, 2.7, [1,10]);
                    
                    
                                                 %calculating current inputs to model
                                                 %based on change
                                                 states = zeros(100,4);
                                                 itss=1;
                                                 for ang_ch = 1:10
                                                     for vel_ch = 1:10
                                                         input_velocity = init_velocity + sampled_velocity_change(vel_ch);
                                                         input_steer_angle_rate = init_steer_angle_rate + sampled_steering_angle_rate_changed(ang_ch);
                    
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
                    
                                                         flag_distant_from_other_points = 0;
                                                         if index == 0
                                                             min_dist_ =1000;
                                                             flag_distant_from_other_points = 1;
                                                             for n = 1:length(valid_nodes)
                                                                 dist_from_n = dist_func(veh_obj,considered_x, considered_y, ...
                                                                     valid_nodes{n}(1), valid_nodes{n}(2));
                                                                 if dist_from_n < min_dist_
                                                                     min_dist_= dist_from_n;
                                                                 end
                                                                 if dist_from_n <= threshold_dist_from_other_points
                                                                     flag_distant_from_other_points = 0;
                    
                                                                 end
                    
                                                             end
                                                           % if flag_distant_from_other_points ==0 
                                                           % 
                                                           %   disp("distance from other nodes very less")  
                                                           % else
                                                           %   disp("min_dist="+string(min_dist_))
                                                           % end
                                                          end
                    
                                                         % scatter(border_1(:,1), border_1(:,2), 5,'red', "filled");
                                                         % hold on 
                                                         % scatter(border_2(:,1), border_2(:,2), 5,'red', "filled");
                                                         % hold on
                                                         % scatter([valid_nodes{1}(1)], [valid_nodes{1}(2)],5, 'green', "filled");
                                                         % hold on
                                                         % scatter([considered_x], [considered_y], 5,'blue', "filled");
                                                         % %saveas(gcf,'imgs/trial/'+string(length(valid_nodes)) + '.png')
                                                         % close
                                                         % % break
                    
                                                         if flag_distant_from_other_points == 1
                                                         %if ismember([target_x target_y], valid_nodes) == false
                                                            %if target point inside road -> this is the next
                                                             %waypoint
                                                             %else discard this point and sample again
                                                             [validity_flag, corresp_road_block]=find_if_target_node_valid(veh_obj, considered_x, considered_y, border_1, border_2,block_threshold, valid_nodes{nearest_node_id}(5));
                    
                                                             % scatter(border_1(:,1), border_1(:,2),5, 'red', "filled");
                                                             % hold on 
                                                             % scatter(border_2(:,1), border_2(:,2),5, 'red', "filled");
                                                             % hold on
                                                             % scatter(start_node{end}(1), start_node{end}(2), 10, 'green', "filled");
                                                             % hold on
                                                             % scatter(considered_x, considered_y, 10, 'blue', "filled");
                    
                                                             %close;
                                                             if validity_flag == true
                                                                 disp("New point found at block ="+string(corresp_road_block))
                                                                 lane_id = find_lane(veh_obj, considered_x, considered_y, corresp_road_block, border_1, border_2, centre_line);
                                                                 valid_nodes{end+1} = [considered_x considered_y considered_head considered_steer corresp_road_block, lane_id];
                                                                 if last_road_block<corresp_road_block
                                                                     last_road_block = corresp_road_block;
                                                                 end
                                                                 if last_road_block==length(border_1) -block_threshold
                                                                     num_goal=num_goal+1;
                                                                 end
                    
                                                                 %tree(nearest_node_id) = length(valid_nodes);
                                                                 tree =[tree; [nearest_node_id length(valid_nodes)]];
                    
                                                                 node_x=[];
                                                                 node_y=[];
                                                                 for l=1:length(valid_nodes)
                                                                      node_x(end+1)=valid_nodes{l}(1);
                                                                      node_y(end+1)=valid_nodes{l}(2);
                                                                 end
                    
                                                                 % scatter(border_1(:,1), border_1(:,2),5, 'red', "filled");
                                                                 % hold on 
                                                                 % scatter(border_2(:,1), border_2(:,2),5, 'red', "filled");
                                                                 % hold on
                                                                 % scatter(node_x, node_y, 5, 'blue', "filled");
                                                                 % % saveas(gcf,'imgs/trial/path'+string(p)+'_'+string(length(valid_nodes)) + '.png')
                                                                 % close
                                                                 flag_find_new_node=1;
                                                                 break
                                                             end
                    
                                                         %check dist from each node <0.5/1.0    
                                                         end
                                                         %might need to break here for
                                                         %100 sampled points
                    
                                                     end
                    
                                             end
                                             if flag_find_new_node == 1
                                                 break
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
                    
                        scatter(border_1(:,1), border_1(:,2),5, 'red', "filled");
                        hold on 
                        scatter(border_2(:,1), border_2(:,2),5, 'red', "filled");
                        hold on
                        scatter(node_x, node_y, 5, 'blue', "filled");
                        saveas(gcf,'imgs/trial/run8_path'+string(p)+'_'+string(length(valid_nodes)) + '.png')
                        close
                        disp("Image stored")
                    
                    pi = 2;
                    
                    filename = 'imgs/tree/run8_path_'+string(p)+'round_'+string(round)+'.mat' ;
                    save(filename,"valid_nodes","tree")
                    
        
                    % results{p} = valid_nodes;
                    % tree_all = [tree_all; tree];
                    end   
        
          pi=2;

                end

           % [results{p}, tree ]= function_sample(veh_obj,border_1, border_2, p, num_paths);
           % tree_all=[tree_all; tree];
      
        end
        
      
       function waypoint_idx = find_path(veh_obj,scenario, tree, tree_nodes, veh_id)
            

            %get maximum calculated block while generating tree
            max_block =0;
            max_id =-1;
            for i=1:length(tree_nodes)
                if max_block <tree_nodes{i}(5)
                    max_block = tree_nodes{i}(5);
                    max_id = i;
                end
            end

            %generate adjacency list
            tree_adj_list ={[-1]};
            for i = 1: height(tree)
                edge = tree(i,:);
                parent =edge(1);
                child = edge(2);
                tree_adj_list{parent}=[tree_adj_list{parent}; [child]];
                if length(tree_adj_list)<child
                    tree_adj_list{end+1} = [-1];
                end
                    

                %end
            end
            
            %store parent child in dict
            queue =[1];
            parent_dict = dictionary;
            parent_dict(1) = -1;

            for i=1:length(tree_adj_list)
                children = tree_adj_list{i};
                for j =1:length(children)
                    if children(j) ~= -1
                        parent_dict(children(j)) = i;
                    end
                end
            end
            %    queue = [queue; children];
            %    processed_node_idx = queue(1);
            %    queue = queue(2:end);
            %    %current_node_is_a_parent = isKey( parent_dict , processed_node_idx );
            %    %if current_node_is_a_parent == 0
            %    if processed_node_idx == max_block
            %        break;
%
            %    end
            %end

            waypoint_idx=[max_id];
            parent_node = parent_dict(max_id);
            while(parent_node ~= -1)
                waypoint_idx=[waypoint_idx; parent_node];
                parent_node = parent_dict(parent_node);
            end
            waypoint_idx=flip(waypoint_idx);
            
            %visualization
            node_x=[];
            node_y=[];
            for i = 1: length(waypoint_idx)
                node_x=[node_x; tree_nodes{waypoint_idx(i)}(1)];
                node_y=[node_y; tree_nodes{waypoint_idx(i)}(2)];
            end

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

            new_border_1=[];
            new_border_2=[];
            for bd = 1:length(border_1)
                 if mod(bd,3) == 0
                     new_border_1=[new_border_1; border_1(bd,:)];
                     new_border_2=[new_border_2; border_2(bd,:)];
                 end
            end
            border_1=new_border_1;
            border_2=new_border_2;
            
            %lane1
            %centre_line=(new_border_1 + new_border_2)/2.0;
            %outer_border=border_2;
            %border_2 = centre_line;

            %multi_Agent
            if veh_id ==1
             centre_line=(new_border_1 + new_border_2)/2.0;
             outer_border=border_2;
             border_2 = centre_line;
            else
             centre_line=(new_border_1 + new_border_2)/2.0;
             outer_border=border_1;
             border_1 = centre_line;
    
            end


            scatter(border_1(:,1), border_1(:,2), 'red', "filled");
            hold on 
            scatter(border_2(:,1), border_2(:,2), 'red', "filled");
            hold on
            %lane1
            scatter(outer_border(:,1), outer_border(:,2), 'red', "filled");
            hold on
            scatter(node_x, node_y, 'blue', "filled");
            %saveas(gcf,'imgs/trial/'+string(length(valid_nodes)) + '.png')
            close
            

            %pi=2;


       end



     

       function veh_obj=vehicle_movement(veh_obj, scenario, veh_id)%iteration, waypoints, speed)

           %rbScenario = roadBoundaries();
           new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
           laneSpecification = lanespec([1 1]);
           road(new_scenario_obj, scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
           rbScenario = roadBoundaries(new_scenario_obj);
           
           num_paths = 10;
           
           %sampled_scenario(veh_obj, scenario, veh_id, num_paths);

           % if veh_id ==1
           %      tree = load ("imgs/tree/path1round_1.mat").tree;
           %      tree_nodes = load("RRT_6_nodes_veh1.mat").valid_nodes;
           % else
           %      tree = load ("RRT_6_tree_veh2.mat").tree;
           %      tree_nodes = load("RRT_6_nodes_veh2.mat").valid_nodes;
           % end

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

           
    
           %need to enable later to visualize scenario
          %speed_total = ones(1, length(waypoints_total))*30;
%
          %while veh_obj.iteration<length(waypoints_total)-1
          %  veh_obj.iteration=veh_obj.iteration+1;
          %  next_position=waypoints_total(veh_obj.iteration, :);
          %  veh_obj.waypoints=[veh_obj.waypoints; next_position];
          %  next_speed=speed_total(veh_obj.iteration);
          %  veh_obj.speed=[veh_obj.speed; next_speed];
          %end
           
       end

       

   end
end