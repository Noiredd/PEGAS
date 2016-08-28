%PM2_createVehicle.m
%VEHICLE PARAMETERS SETUP SHEET TEMPLATE

%FIRST STAGE
stage_m0       = 97198;             %launch mass [kg]
stage_thrust   = 1217150;           %thrust ASL [N]
stage_isp0     = 250;               %isp vacuum [s]
stage_isp1     = 230;               %isp asl [s]
stage_fuel     = 75744;             %stage 1 fuel mass [kg]
stage_area     = 7.06;              %reference cross section area [m2]
stage_drag     = [ 0.0   0.122;
                   256.0 0.122;
                   343.2 1.263;
                   643.5 1.058;
                   909.5 1.154;
                   1673  0.776;
                   9999  0.776; ];  %drag coefficient in relation to velocity [m/s -]
                                    %TODO: how to deal with this? any way to get it directly from FAR plot?
stage_dm       = stage_thrust/...
                  (stage_isp1*g0);  %mass flow rate [kg/s]
stage_time     = stage_fuel*stage_isp1*g0/...
                      stage_thrust; %burn time [s]
stage_spoolup  = 1.4;               %engine ignition precedes release by [s]
stage = struct('m0', stage_m0-stage_spoolup*stage_dm,...
               'dm', stage_dm,...
               'i0', stage_isp0,...
               'i1', stage_isp1,...
               'mt', stage_time-stage_spoolup,...
               'ra', stage_area,...
               'dc', stage_drag);
oldicbm(1) = stage;

%SECOND STAGE
stage_m0       = 7442;              %stage 2 mass at separation [kg]
stage_thrust   = 55400;             %thrust vacuum [N]
stage_isp0     = 340;               %isp vacuum [s]
stage_isp1     = 340;               %isp ASL [s]
stage_fuel     = 6284;              %stage 2 fuel mass [kg]
stage_A        = 7.06;              %reference cross section area [m2]
stage_drag     = [999 0.5; 9999 0.4; ]; %will mostly run in vacuum anyway
stage_dm       = stage_thrust/...
                   (stage_isp0*g0); %mass flow rate [kg/s]
stage_time 	= stage_fuel*stage_isp0*g0/...
                      stage_thrust; %max burn time [s]
stage = struct('m0', stage_m0,...
               'dm', stage_dm,...
               'i0', stage_isp0,...
               'i1', stage_isp1,...
               'mt', stage_time,...
               'ra', stage_area,...
               'dc', stage_drag);
oldicbm(2) = stage;

clearvars stage_m0 stage_thrust stage_isp0 stage_isp1 stage_fuel stage_area
clearvars stage_drag stage_dm stage_time stage_spoolup stage

disp('Note for newcomers: you can proceed right to real_test.m and see what it can do, however, you must load the pitch program from pp_prog.mat and create a variable s1_prog by transposing the former.');