# autonomouse_car
MATLAB code for wall-following autonomous robot car

Our objective is to implemet PD controller in MATLAB with ROS and LIDAR data. With calculated and observed data, control autonomous robot car to follow its path while keeping the distance between itself and wal regulally. Autonomous car will also change its direction and speed based on its observation during the operation. 

Settings
Distance between wall: 5cm

Forward stopping distance: 4cm

Linear speed: θ < 5°, v = 0.4m/s
               5° ≤ θ ≤ 10°, 0.25 < v < 0.4m/s
               θ > 10°, v = 0.25m/s
               
K_p & K_d values can be varies based on calculations

Lidar directions are based on the idex increment direction in counter-clock wise

Used formulas 
a = tan^-1 [acos(θ-b)/asin(θ)]
    where a = lidar direction
    
PD formula: u_t = (K_p)(e_t) + (K_d)[(e_t - e_t-1)/change in t]
            where e_t = prev_error
            
Converting linear velocity to angular velocity: L/tan(u) = v/u
                                                where u = angular velocity
