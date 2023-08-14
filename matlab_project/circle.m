
%Number of robots
N = 4;

%Set positions
initial_positions = [-0.5, 0.5, 0, 0; 0, 0, -0.5, 0.5; 0*pi/180, 180*pi/180, 90*pi/180, -90*pi/180];
final_positions = [sqrt(2)/4, -sqrt(2)/4, sqrt(2)/4, -sqrt(2)/4; -sqrt(2)/4, sqrt(2)/4, sqrt(2)/4, -sqrt(2)/4; 0*pi/180, 180*pi/180, 90*pi/180, -90*pi/180];

%Robotarium object
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

si_barrier_certificate = create_si_barrier_certificate_with_boundary();
si_to_uni_dynamics = create_si_to_uni_dynamics();

%iterations = 1000;

k = 0.1;
x = r.get_poses();
r.step();

while(norm(x(1:2,:) - final_positions(1:2,:)) > 0.03)
    x = r.get_poses();
    v = k * (final_positions(1:2,:) - x(1:2,:));
    v = si_barrier_certificate(v, x(1:2, :));
    vu = si_to_uni_dynamics(v, x);
    
    r.set_velocities(1:N, vu);
    r.step();
end

r.debug();