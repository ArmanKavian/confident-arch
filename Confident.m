%% Confident

manual = 0;

%% Init path following controller
controller = robotics.PurePursuit;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 1;
controlRate = robotics.Rate(10);

%Init simulator
robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('complexMap',2);
robot.enableLaser(true);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);

%%Init PRM
mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 300;
prm.ConnectionDistance = 10;

%%
startLocation = [6.0 2.0];
endLocation = [16.0 18.0];
path = findpath(prm, startLocation, endLocation);
%show(prm, 'Map', 'off', 'Roadmap', 'off');

release(controller);
controller.Waypoints = path;

robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robot.setRobotPose(robotCurrentPose);

distanceToGoal = norm(robotCurrentLocation - robotGoal);

goalRadius = 0.2;

%vfh
vfh = robotics.VectorFieldHistogram;

last_cmd = 'n';
c_op = .5;

vt = 0;
wt = 0;

last_v_op = 0;
last_w_op = 0;

d_op = 0;
has_cmd = 0;

%Control loop
reset(controlRate);
while( distanceToGoal > goalRadius )    
    
    %Keyboard input
    f = figure(1);
    set(f,'KeyPressFcn',@(h_obj,evt) setappdata(f, 'cmd', evt.Key));
    cmd = getappdata(f, 'cmd');
    
    %Manual operation behaviour
    v_op = 0;
    w_op = 0;
    if ~isempty(cmd)
        setappdata(f, 'cmd', []);
        
        switch cmd
            case 'w'
                v_op = 3;
                w_op = 0;
            case 's'
                v_op = -2;
                w_op = 0;
            case 'a'
                w_op = .8;
            case 'd'
                w_op = -.8;
        end
        
        has_cmd = 1;
    else
        if abs(last_w_op) > .2
            w_op = last_w_op - sign(last_w_op) * .2;
        end
        
        if abs(last_v_op) > .2
            v_op = last_v_op - sign(last_v_op) * .2;
        end
        
        has_cmd = 0;
    end
    
    if last_cmd == cmd
        c_op = c_op + .2;
    elseif (cmd ~= 'n')
        c_op = c_op + .07;
    else
        c_op = .5;    
    end
    
    last_v_op = v_op;
    last_w_op = w_op;
    
    %Path following behaviour
    [v_pf, w_pf] = step(controller, robot.getRobotPose);
    c_pf = .6;
    
    %Obstacle avoidance behaviour
    if (has_cmd)
        targetDir = asin(w_op/2);
    else
        targetDir = asin(w_pf/2);
    end
    
    ranges = robot.LaserSensor.AngleSweep;
    ranges(ranges<0) = 0;
    angles = linspace(0,180,21);   
    steeringDir = step(vfh, ranges, angles, targetDir);
	if ~isnan(steeringDir) 
        v_oa = 0.2;
		w_oa = exampleHelperComputeAngularVelocity(steeringDir) * 5;
    else
		v_oa = 0.0;
		w_oa = 0.7;
    end
    
    obs = ranges(7:13);
    near = obs(obs > 0);
    if ~isempty(near)
        min_dist = min(near);
        if (min_dist < .6)
            c_oa = 1;
        elseif (min_dist < .1)
            c_oa = .8;
        elseif (min_dist < 1.2)
            c_oa = .3;
        else
            c_oa = .2;
        end
    else
        c_oa = .3;
    end

    %c_oa = 1;
    
    %Arbitrator
    if (has_cmd)
        p_op = .8;
        p_pf = .1;
        p_oa = .6;
    else
        p_op = .2;
        p_pf = .8;
        p_oa = .7;
    end
    
    o_op = p_op * w_op * c_op;
    o_pf = p_pf * w_pf * c_pf;
    o_oa = p_oa * w_oa * c_oa;
    
    if manual == 0
        vt = mean([v_pf v_oa v_op]);
        wt = mean([o_op o_pf o_oa]);
    else
        vt = v_op;
        wt = o_op;
    end
    
    drive(robot, vt, wt);
    
    %Update
    robotCurrentPose = robot.getRobotPose();
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    waitfor(controlRate);
end

%% Shutdown
drive(robot, 0, 0);
delete(robot);

