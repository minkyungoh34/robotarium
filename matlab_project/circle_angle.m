%Number of robots
N = 9;

%Set positions
radius = 0.8; %radius
offset_angle = (45*pi)/180; %offset angle in radians

%Angles and xy positions
thetai = (2*pi) / (N); %initial angle calculation

starting_angle = [1:N]*thetai;

xi = cos(starting_angle) * radius; %initial x position calculation
yi = sin(starting_angle) * radius;%initial y position calculation


thetaf = (starting_angle + pi + offset_angle); %final angle for the robots (move by 45 degrees to the opposite side)
xf = cos(thetaf) * radius; %final x position calculation
yf = sin(thetaf) * radius; %final y position calculation

%How do I make these two matrices applicable to all the robots?
initial_positions = [xi; yi; starting_angle + pi]; %initial position matrix 
final_positions = [xf; yf; thetaf + pi];%final position matrix

%Robotarium object initialization - number of robots, show simulation, and
%set initial positions
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%barriers ??
si_barrier_certificate = create_si_barrier_certificate_with_boundary();
si_to_uni_dynamics = create_si_to_uni_dynamics();

k = 0.1; %?
x = r.get_poses(); %Get the recent poses of the robots 
r.step(); %Send the velocity to the robots

%Algorithm
while(norm(x(1:2,:) - final_positions(1:2,:)) > 0.03) %Check if the robots are close enough to the goal (0.03)
    x = r.get_poses(); %Get the most recent poses of the robots
    v = k * (final_positions(1:2,:) - x(1:2,:)); %velocity calculation depending on distance from the final position?
    v = si_barrier_certificate(v, x(1:2, :)); %velocity dependingo on closeness to other robots?
    vu = si_to_uni_dynamics(v, x); %unicycle dynamics?
    
    r.set_velocities(1:N, vu); %set velocity (linear and angular) of the robots (1-N)
    r.step(); %Send the velocity to the robots 
end

%Print out simulation issues
r.debug();