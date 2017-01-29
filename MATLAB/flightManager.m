%flightManager.m
%Function inputs a vehicle and a target as well as several additional
%values and performs a complete launch, switching stages when necessary and
%handling initializations. Outputs two lists: one of powered stage results
%and another of coast stages.
function [flight] = flightManager(vehicle, init, target, dt, s1guidance, upfgCycle, coastLength)
    n = length(vehicle);
    %Handle first stage first, as it's guided with a separate scheme and outputs different results struct.
    powered(1) = flightSim3D(vehicle, 1, init, s1guidance, dt);
    %Create UPFG and coast handlers
    upfg_control = struct('type', 3, 'target', target, 'major', upfgCycle);
    coast_control = struct('type', 5, 'length', coastLength);
    %Handle the rest of the flight in a loop
    for i=2:n
        %Things work a bit differently if zero coast period was specified.
        if coastLength>0
            coast(i-1) = flightSim3D(vehicle, i, resultsToInit(powered(i-1)), coast_control, dt);
            powered(i) = flightSim3D(vehicle, i, resultsToInit(coast(i-1)), upfg_control, dt);
        else
            coast = [];
            powered(i) = flightSim3D(vehicle, i, resultsToInit(powered(i-1)), upfg_control, dt);
        end
        %If a powered stage was cut off by the guidance algorithm - do not
        %simulate any more stages.
        if powered(i).ENG>1 && i<n
            n = length(powered);
            fprintf('Used less stages than available (%d).\n', i);
            break;
        end
        %Support crash detection
        if powered(i).ENG<-10
            fprintf('Critical mission failure. Vehicle crashed (stage %d).\n', i);
            break;
        end
    end
    if ~exist('coast','var')
        coast = [];
    end
    %Output a struct
    flight = struct('powered', powered, 'coast', coast, 'n', n);