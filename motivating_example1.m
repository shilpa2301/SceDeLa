%drivingScenarioDesigner;

[scenario, ego]=overlapping_scenario_simulink();
plot(scenario);
while advance(scenario)
  pause(0.001)
end

