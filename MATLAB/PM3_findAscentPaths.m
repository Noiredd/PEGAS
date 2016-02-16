%PM3_findAscentProfile.m
%Finds gravity turn parameters (initial velocity, initial pitchover) that
%produce specified apoapsis of the osculating orbit after 1st stage burnout.
%Sweeps 2D parameter space of possible gravity turns. Performs a section of
%a resulting surface on a given constraint (desired apoapsis at burnout),
%creating a 1-D solution. Approximates this solution with a linear function
%and displays a plot, allowing the user to select their solution point.
%Dependencies:
%   sweepGravityTurn.m
%PARAMETERS - USER INPUT
ap_dt = 0.1;    %simulation precision
ap_vmin = 50; ap_vmax = 80; ap_vdv = 10;  %min/max velocity to swep, precision
ap_pmin = 1;  ap_pmax = 7;   ap_pdp = 2; %same for pitchover value
ap_target = 150;    %target apoapsis to find equation for
%input vectors and their lengths
ap_v = ap_vmin:ap_vdv:ap_vmax;
ap_p = ap_pmin:ap_pdp:ap_pmax;
ap_n = length(ap_v);
ap_m = length(ap_p);
%actual sweep - a very computationally heavy function
[ap_a,ap_q] = sweepGravityTurn(s1_vehicle, s1_init, ap_v, ap_p, ap_dt);
%for visualisation data must be converted to a proper format
ap_x = zeros(1,ap_n*ap_m);
ap_y = zeros(1,ap_n*ap_m);
ap_z = zeros(1,ap_n*ap_m);
for i = 1:ap_n
    for j = 1:ap_m
        ap_x((i-1)*ap_m+j) = ap_v(i);
        ap_y((i-1)*ap_m+j) = ap_p(j);
        ap_z((i-1)*ap_m+j) = ap_a(j,i);
    end;
end;
%display calculated surface (domain)
figure(1); clf;
surf(ap_v,ap_p,ap_a);
title('Sweep results surface');
%simple check: are there definitely no solutions?
if max(ap_z)<ap_target
    disp('Target apoapsis not reached!')
    break;
end;
if min(ap_z)>ap_target
    disp('Target apoapsis overshot!')
    break;
end;
%slice at solution value (obtains 1D solution curve)
figure(2);
[ap_C,ap_h] = contour(ap_v,ap_p,ap_a, [ap_target ap_target]);   %extract contour
set(ap_h,'LevelList',ap_target);
ap_xc = ap_C(1,2:length(ap_C)); %extracted contour data
ap_yc = ap_C(2,2:length(ap_C)); %for some reason (:,1) is a weird value
figure(2); clf; hold on;
plot(ap_xc,ap_yc,'green');
%fit linear equation to the data, to obtain simple p=m*v+b function
ap_cvars = polyfit(ap_xc,ap_yc,1);
ap_cm = ap_cvars(1)     %CRUCIAL PARAMETERS HERE
ap_cb = ap_cvars(2)
xf = min(ap_xc):0.1:max(ap_xc); %for visualisation purposes only
yf = xf*ap_cm+ap_cb;
plot(xf,yf,'blue');
title('Altitude solution curve: pitch~velocity');
legend('original surface contour', 'linear approximation', 'Location', 'SouthEast');
hold off;
%secondary parameter solution
%not really useful since there's a correlation between maxQ and drag losses
%and the differences are smaller than interpolation error
%figure(3); clf;
%[ap_V,ap_P] = meshgrid(ap_vmin:ap_vdv:ap_vmax, ap_pmin:ap_pdp:ap_pmax);
%ap_2 = griddedInterpolant(ap_V', ap_P', ap_q');
%ap_2c = ap_2(ap_xc,ap_yc);
%plot(ap_xc, ap_2c);
%title('Secondary constraint surface');