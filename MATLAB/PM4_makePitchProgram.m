%PM4_makePitchProgram.m
%Creates a pitch vs time table by linear approximation of a natural gravity
%turn path under parameters chosen from equation found previously.
%Dependencies:
%   createLineApprox.m
%       linearFit.m
%   flightSim2D.m       legacy 2D simulation, because faster than 3D
%                       consider complete migration to 3D for consistency
%PARAMETERS - USER INPUT
pp_v = 55;  %velocity to start pitchover at
pp_n = 5;   %linear approximation control points for the free trajectory
            %(results in n-1 segments)
            %whole trajectory will have n+3 control points however (and
            %n+2 segments, respectively)
pp_k = 10;  %kill rotation period - final value will be reached 'pp_k'
            %earlier and retained until the end of program
%calculate pitchover angle
pp_p = pp_v*ap_cm+ap_cb;
%generate a natural gravity turn for given parameters
r = flightSim2D(s1_vehicle, s1_init, struct('type',0, 'p', pp_p, 'v', pp_v), ap_dt);
%prepare variables and call the approximation function
clear pp_pitch;
clear pp_prog;
pp_prog = createLineApprox([r.Plots.t;r.Plots.pitch], pp_n);
%killrot
pp_prog(:,pp_n+4) = [pp_prog(1,pp_n+3); pp_prog(2,pp_n+3)];
pp_prog(1,pp_n+3) = pp_prog(1,pp_n+3)-pp_k;
%plot and compare results
figure(3); clf; hold on;
plot(0.1:0.1:length(r.Plots.pitch)/10, r.Plots.pitch, 'green');
plot(pp_prog(1,:), pp_prog(2,:), 'blue');
title('Linearised pitch program');
legend('original gravity turn pitch', 'linear approximated pitch', 'Location', 'SouthEast');
hold off;
%cleanup
clearvars pp_v pp_n pp_k pp_p r pp_pitch;