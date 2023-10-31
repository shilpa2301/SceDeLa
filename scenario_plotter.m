classdef scenario_plotter% < Vehicle
   properties
       scenario_inst
   end

   methods
       function scenario_obj=scenario_plotter()
            %scenario_obj.scenario_inst=scenario_with_ego_veh;
            scenario_obj.scenario_inst=drivingScenario;%smoothTrajectory(scenario_with_ego_veh.ActorSpecifications, veh_actor_trajectory_params.waypoints, veh_actor_trajectory_params.speed);
       end
       function plot_scenario( scenario_with_ego_veh, DSD_Scenario, veh_actor_trajectory_params1, veh_actor_trajectory_params2)

              % Convert the 1x1 struct to a drivingScenario object
              %road1.mat
              %scenario = drivingScenario('GeographicReference', [42.29969 -71.35044 0], ...
    %'VerticalAxis', 'Y');
              scenario_obj = drivingScenario('VerticalAxis', 'Y');
              
              % Add road data (assuming road_data is a field in the struct)
              %scenario_obj.road = scenario_with_ego_veh.scenario_inst.RoadSpecifications;
              %scene=drivingScenarioDesigner(DSD_Scenario);

              laneSpecification = lanespec([1 1]);
              road(scenario_obj, DSD_Scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');

              % Add other actors (assuming actor_data is a field in the struct)
             
              egoVehicle=vehicle(scenario_obj);
              advVehicle=vehicle(scenario_obj);
              
              smoothTrajectory(egoVehicle, veh_actor_trajectory_params1.waypoints, veh_actor_trajectory_params1.speed);
              smoothTrajectory(advVehicle, veh_actor_trajectory_params2.waypoints, veh_actor_trajectory_params2.speed);
              
              % Plot the driving scenario
              plot(scenario_obj, 'Centerline','on','RoadCenters','on','Waypoints','on');
              while advance(scenario_obj)
                pause(0.1)%0.001)
              end

                 
       end
   end
end