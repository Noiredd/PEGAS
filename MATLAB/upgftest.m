%vehicle definition as in PM2 file, target and state structs just examples
vehicle = struct('thrust', 55400, 'isp', 340, 'mass', 7442);
target = struct('radius', R+250000, 'velocity', 7850, 'angle', 0, 'normal', [0,0,1]);
state = struct('time', 0, 'mass', 7442, 'radius', [R+100000,0,0], 'velocity', [2000,2500,0]);
%arbitrary preparation of first guesses on rd and vgo
rdinit = state.radius+0.15*cross(target.normal,state.radius);
ix = rdinit/norm(rdinit);
rdinit = ix*target.radius;
iz = cross(ix,target.normal);
vdinit = (target.velocity*[ix;target.normal;iz]*[0;0;1])' - state.velocity;
previous = struct('time', 0, 'tgo', 0, 'rbias', [0,0,0], 'rd', rdinit, 'rgrav', mu/2*state.radius/norm(state.radius)^3, 'vgo', vdinit);

N=15;
vh=zeros(1,N);
th=zeros(1,N);
for i=1:N
    previous = unifiedPoweredFlightGuidance(vehicle,target,state,previous);
    th(i) = previous.tgo;
    vh(i) = norm(previous.vgo);
end

figure(4); clf; hold on; plot(vh, 'b'); plot(th, 'g'); hold off;
plots=0;
if plots==3
    figure(1);
    clf;
    hold on;
    %earth_sphere('m');
    temp = zeros(2,3);
    temp(2,:) = r;
    plot3(temp(:,1), temp(:,2), temp(:,3), 'k');
    temp(2,:) = rd1;
    plot3(temp(:,1), temp(:,2), temp(:,3), 'y');
    temp(2,:) = rp;
    plot3(temp(:,1), temp(:,2), temp(:,3), 'r');
    temp(1,:) = r;
    temp(2,:) = rthrust;
    plot3(temp(:,1), temp(:,2), temp(:,3), 'b');
    hold off;
elseif plots~=0
    figure(2);
    clf;
    hold on;
    temp = zeros(2,2);
    temp(2,:) = r(1:2);
    plot(temp(:,1), temp(:,2), 'k');
    temp(2,:) = rd1(1:2);
    plot(temp(:,1), temp(:,2), 'y');
    temp(2,:) = rp(1:2);
    plot(temp(:,1), temp(:,2), 'r');
    temp(1,:) = r(1:2);
    temp(2,:) = rthrust(1:2);
    plot(temp(:,1), temp(:,2), 'b');
    temp(2,:) = rthrust2(1:2);
    plot(temp(:,1), temp(:,2), 'k');
    hold off;
end