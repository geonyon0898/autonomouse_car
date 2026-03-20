scan_angle_a = deg2rad(315);       
scan_angle_b = deg2rad(270);       
theta        = abs(scan_angle_a - scan_angle_b);   

lookahead_distance  = 0.4;         
target_distance     = 0.5;  
car_wheelbase       = 0.165;       

k_p = 1.2;
k_d = 0.005;
prev_error = 0;
last_time  = seconds(rostime('now'));

laser    = rossubscriber("/scan","DataFormat","struct");
robotCmd = rospublisher("/cmd_vel","DataFormat","struct");
velMsg   = rosmessage(robotCmd);

while true
    [scan, flag] = receive(laser, 5);
    Lidscan = rosReadLidarScan(scan);
   
    if (Lidscan.Rangs(1) <= lookahead_distance || flag == 0)
        velMsg.Linear.X  = 0.0;
        velMsg.Angular.Z = 0.0;
        send(robotCmd, velMsg);
        break;
    end

    index_a = floor(scan_angle_a / scan.AngleIncrement);
    index_b = floor(scan_angle_b / scan.AngleIncrement);

    distance_a = Lidscan.Ranges(index_a);
    distance_b = Lidscan.Ranges(index_b);

    if isinf(distance_a) || isnan(distance_a) || isinf(distance_b) || isnan(distance_b)
        velMsg.Linear.X  = 0.25;
        velMsg.Angular.Z = 0.0;

    else 
         alpha_n = distance_a * cos(theta) - distance_b;
         alpha_d = distance_a * sin(theta);

         alpha = atan(alpha_n / alpha_d);
    end

    d     = distance_b * cos(alpha);
    d_bar = d + lookahead_distance * sin(alpha);

    error = target_distance - d_bar;
    now_time   = seconds(rostime('now'));
    delta_time = now_time - last_time;

    d_error        = (error - prev_error) / delta_time;
    steering_angle = k_p * error + k_d * d_error;

    prev_error = error;
    last_time  = now_time;

    if (abs(steering_angle) < deg2rad(5))
        velMsg.Linear.X = 0.40;
    elseif (abs(steering_angle) < deg2rad(10))
        velMsg.Linear.X = 0.30;
    else
        velMsg.Linear.X = 0.25;
    end

    velMsg.Angular.Z = velMsg.Linear.X * tan(steering_angle) / car_wheelbase;

    send(robotCmd, velMsg);
end
