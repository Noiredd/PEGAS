%PM5_testAscentProfile.m
%Ascent simulation using precalculated pitch program.
%PARAMETERS - USER INPUT
s1_dt = 0.05;           %sim precision [s]
%run simulation and show Plots
s1_prog = pp_prog';
s1_results = ascentSimulation(struct('type', 1, 'program', s1_prog), s1_dt);
figure(4); clf;
subplot(2,3,1); plot(s1_results.Plots.t, s1_results.Plots.pitch); title('Pitch');
subplot(2,3,4); plot(s1_results.Plots.t, s1_results.Plots.angle); title('Flight angle');
subplot(2,3,[2;5]); plot(s1_results.Plots.t, s1_results.Plots.altitude); title('Altitude');
subplot(2,3,3); plot(s1_results.Plots.t, s1_results.Plots.vy); title('Vy');
subplot(2,3,6); plot(s1_results.Plots.t, s1_results.Plots.vx); title('Vx');