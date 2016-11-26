clearvars vehicle
%Atlas V 531
%   3x Aerojet SRB + 1x RD-180
%   RD-180 (Common Core Booster)
%   RL-10C-1 (Centaur)
%Data taken from:
%   http://spaceflight101.com/spacerockets/atlas-v-531/
%   http://www.astronautix.com/a/atlasv.html    (when in doubt)
payload = 15575;    %certified max to LEO

stage_mode = 1;
stage_m0 = (21054+284089+285) + ...     %CCB + fuel + interstage
            3 * 46697 + ...             %SRBs
            (2243+20830) + 4379 + ...   %Centaur + long PLF
            payload;
stage_thrust = 3827000 + 3*1688400;
stage_isp1 = 272.4;                     %estimated 249s for SRBs
stage_isp0 = 301.2;
stage_time = 94;
stage_area = pi*(5.4/2)^2;
stage_drag = [ 0.0  0.08;
               250  0.08;
               343  0.80;
               999  0.50;
               9999 0.40; ];            %not supported by any real data!
stage_dm = stage_thrust / (stage_isp1*g0);          %total mass flow rate [kg/s]
stage = struct('SM', stage_mode,...
               'aL', 0,...
               'm0', stage_m0,...
               'dm', stage_dm,...
               'i0', stage_isp0,...
               'i1', stage_isp1,...
               'mt', stage_time,...
               'ra', stage_area,...
               'dc', stage_drag);
vehicle(1) = stage;

stage_mode = 1;
stage_m0 = (21054+284089+285) + ...     %CCB + fuel + interstage
            (2243+20830) + 4379 + ...   %Centaur + long PLF
            payload;
stage_thrust = 3827000;
stage_isp1 = 311;
stage_isp0 = 338;
stage_dm = stage_thrust / (stage_isp1*g0);
stage_m0 = stage_m0 - stage_time*stage_dm;  %that much was burnt through stage 1
stage_time = 284089/stage_dm - 94;      %total capability minus time already spent
stage_area = pi*(5.4/2)^2;
stage_drag = [ 0.0  0.08;
               250  0.08;
               343  0.80;
               999  0.50;
               9999 0.40; ];
stage = struct('SM', stage_mode,...
               'aL', 0,...
               'm0', stage_m0,...
               'dm', stage_dm,...
               'i0', stage_isp0,...
               'i1', stage_isp1,...
               'mt', stage_time,...
               'ra', stage_area,...
               'dc', stage_drag);
vehicle(2) = stage;

stage_mode = 1;
stage_m0 = 2243+20830 + payload;        %Centaur + payload
stage_thrust = 101800;
stage_isp1 = 449.7;
stage_isp0 = 449.7;
stage_dm = stage_thrust / (stage_isp1*g0);
stage_time = 20830/stage_dm;
stage_area = pi*(3.05/2)^2;
stage_drag = [ 0.0  0.08;
               250  0.08;
               343  0.80;
               999  0.50;
               9999 0.40; ];
stage = struct('SM', stage_mode,...
               'aL', 0,...
               'm0', stage_m0,...
               'dm', stage_dm,...
               'i0', stage_isp0,...
               'i1', stage_isp1,...
               'mt', stage_time,...
               'ra', stage_area,...
               'dc', stage_drag);
vehicle(3) = stage;


clearvars payload
clearvars stage_mode stage_m0 stage_thrust stage_isp0 stage_isp1 stage_time stage_area stage_drag stage_dm stage_accLim stage_fuel stage