clear all;
close all;

scenario=drivingScenario;%()
%scenario_to_plot=drivingScenarioDesigner('road1.mat')
DSD_Scenario=load("road1.mat").data;
veh_obj=Vehicle(DSD_Scenario);
%scene=load("road1.mat").data;
%scenario = drivingScenario(scene, 'VerticalAxis', 'Y');
veh_obj=vehicle_movement(veh_obj, DSD_Scenario);

% put this to function later
%new_scenario_obj = drivingScenario('VerticalAxis', 'Y');
%laneSpecification = lanespec([1 1]);
%road(new_scenario_obj, DSD_Scenario.RoadSpecifications.Centers, 'Lanes', laneSpecification, 'Name', 'Apple Hill Drive');
%rbScenario = roadBoundaries(new_scenario_obj);
%
%for i=1:length(rbScenario)
%    
%end

%veh_actor=vehicle(scenario, ...
%    'ClassID', 1, ...
%    'Position', [veh_obj.waypoints(1,:)], ...
%    'Mesh', driving.scenario.carMesh, ...
%    'Name', 'Car');

veh_actor=vehicle(scenario, ...
    'ClassID', 1, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
DSD_Scenario.ActorSpecifications=veh_actor;
veh_actor_Waypoints=veh_obj.waypoints;
veh_actor_Speed=veh_obj.speed;
veh_actor_trajectory_params=struct('waypoints',veh_actor_Waypoints,'speed', veh_actor_Speed);
%smoothTrajectory(veh_actor, veh_actor_Waypoints, veh_actor_Speed);
scenario_generation=scenario_plotter();
scenario_generation.plot_scenario( DSD_Scenario,veh_actor_trajectory_params);
