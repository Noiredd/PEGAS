%flightManager.m
%Function inputs a vehicle and a target as well as several additional
%values and performs a complete launch, switching stages when necessary and
%handling initializations. Outputs two lists: one of powered stage results
%and another of coast stages.
function [flight] = flightManager(vehicle, init, target, dt, s1guidance, upfgCycle, coastLength, events)
    global R;
    n = length(vehicle);
    %Handle first stage first, as it's guided with a separate scheme and outputs different results struct.
    powered(1) = flightSim3D(vehicle, 1, init, s1guidance, events, dt);
    %Create UPFG handler
    upfg_control = struct('type', 3, 'target', target, 'major', upfgCycle);
    %If coast length was given by a single number, we will create an ad-hoc
    %array, so the rest of the code was more unified.
    if length(coastLength)==1
        coast_ = coastLength;
        for i=1:n
            coastLength(i) = coast_;
        end
    %If less coasts lengths were passed than needed (n-1 for n stages),
    %will replicate the last element enough times.
    elseif length(coastLength)<n-1
        for i=length(coastLength):n
            coastLength(i) = coastLength(length(coastLength));
        end
    end
    %Separate iterator for coast phases - if some are zero-long, their
    %structs will not be created, so they and powered stages will be uneven.
    c = 0;
    %Handle the rest of the flight in a loop
    for i=2:n
        %If then next coast period has non-zero length, handle it and
        %continue the flight. Otherwise just fly a powered phase from the
        %last one.
        if coastLength(i-1)>0
            %Iterate coast phases.
            coast_control = struct('type', 5, 'length', coastLength(i-1));
            c = c + 1;
            coast(c)   = flightSim3D(vehicle, i, resultsToInit(powered(i-1)), coast_control, events, dt);
            powered(i) = flightSim3D(vehicle, i, resultsToInit(coast(c)), upfg_control, events, dt);
        else
            powered(i) = flightSim3D(vehicle, i, resultsToInit(powered(i-1)), upfg_control, events, dt);
        end
        %If a powered stage was cut off by the guidance algorithm - do not
        %simulate any more stages.
        if powered(i).ENG>1 && i<n
            n = length(powered);
            fprintf('Used less stages than available (%d of %d).\n', i, length(vehicle));
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