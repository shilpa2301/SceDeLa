function cost = costFunction(wp,obj,state,tspan,inputs)
    [~,y] = ode45(@(t,y) derivative(obj,y,inputs),tspan,state);
    cost = norm(y(end,:)-wp,2);
end