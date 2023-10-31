clear all;
close all;

scenario=drivingScenario;%()
%scenario_to_plot=drivingScenarioDesigner('road1.mat')
DSD_Scenario=load("road1.mat").data;
veh_obj1=Vehicle(DSD_Scenario);
veh_obj1=vehicle_movement(veh_obj1, DSD_Scenario, 1);

veh_obj2=Vehicle(DSD_Scenario);
veh_obj2=vehicle_movement(veh_obj2, DSD_Scenario,2);

veh_actor1=vehicle(scenario, ...
    'ClassID', 1, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
veh_actor2=vehicle(scenario, ...
    'ClassID', 2, ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
DSD_Scenario.ActorSpecifications=veh_actor1;
veh_actor_Waypoints1=veh_obj1.waypoints;
veh_actor_Speed1=veh_obj1.speed;
veh_actor_trajectory_params1=struct('waypoints',veh_actor_Waypoints1,'speed', veh_actor_Speed1);

DSD_Scenario.ActorSpecifications=veh_actor2;
veh_actor_Waypoints2=veh_obj2.waypoints;
veh_actor_Speed2=veh_obj2.speed;
veh_actor_trajectory_params2=struct('waypoints',veh_actor_Waypoints2,'speed', veh_actor_Speed2);

scenario_generation=scenario_plotter();
scenario_generation.plot_scenario( DSD_Scenario,veh_actor_trajectory_params1,veh_actor_trajectory_params2);
