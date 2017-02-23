function [result] = vehicleTools(task, stage, mode, input)
%VEHICLETOOLS calculate often-used parameters for a given stage and mode
%(constant thrust of constant acceleration). Exact input depends on type of
%call. Constant acceleration mode can only be used for single-engine stages
%(collapse multiple engines with the same parameters into one) that are
%throttleable (ie. no thrust profile allowed for constant acceleration).
%The function will not check for violation of throttle limits (will warn if
%they are exceeded though). Constant acceleration assumes vacuum conditions
%for Isp calculation.
%
%COMMON INPUT
%    stage      Struct of type vehicle stage (ie. an element of a vehicle
%               array).
%    mode       Mode to be used; allowed choices:
%                   1 - constant thrust
%                   2 - constant acceleration
%
%
%time = VEHICLETOOLS('tgo', stage, mode, fuel)
%Calculates maximum burn time for a stage with given set of engines and
%total mass of fuel.
%
%INPUT
%    fuel       If mode==1: Mass of fuel to be burned (kg).
%               If mode==2: Array of length 2, containing the following:
%                               fuel(1) = mass of fuel to be burned (kg).
%                               fuel(2) = acceleration limit (G).
%
%OUTPUT
%    time       Time it takes to burn the given amount of fuel.
%
%
%mass = VEHICLETOOLS('mass', stage, mode, time)
%Calculates mass of fuel burned by a stage with given set of engines over
%some given time.
%
%INPUT
%    time       If mode==1: Time of burn (s).
%               If mode==2: Array of length 2, containing the following:
%                               time(1) = time of burn (s).
%                               time(2) = acceleration limit (G).
%OUTPUT
%    mass       Mass of fuel burned over the given time.

    result = 0;
    switch task
        case 'tgo'
            if mode==1      %constant-thrust
                dm = 0;
                for i=1:length(stage.engines)
                    if stage.engines(i).mode~=1
                        fprintf('vehicleTools couldnt calculate tgo for the given stage. One of the engines is in profiled or unknown mode.\n');
                        return
                    end
                    dm = dm + stage.engines(i).flow;
                end
                result = input / dm;
            elseif mode==2  %constant-acceleration
                if length(stage.engines)~=1
                    fprintf('vehicleTools couldnt calculate tgo for the given stage. Only single-engined stages can run in constant-acceleration mode.\n');
                    return
                end
                m0 = stage.m0;
                isp = stage.engines(1).isp0;
                result = isp / input(2) * log( m0/(m0-input(1)) );
                %check for violation of engine throttling limits
                maxFlow = input(2) * m0 / isp;
                minFlow = input(2) * (m0-input(1)) / isp;
                nominalFlow = stage.engines(1).flow;
                minThrottle = stage.engines(1).data(1);
                maxThrottle = stage.engines(1).data(2);
                if maxFlow > maxThrottle*nominalFlow || minFlow > maxThrottle*nominalFlow
                    fprintf('vehicleTools found a violation of upper throttle limit! Time calculated by this call will likely underestimate true burn time.\n');
                end
                if minFlow < minThrottle*nominalFlow || maxFlow < minThrottle*nominalFlow
                    fprintf('vehicleTools found a violation of lower throttle limit! Time calculated by this call will likely overestimate true burn time.\n');
                end
            else
                fprintf('vehicleTools couldnt calculate tgo for the given stage. Unknown throttle mode.\n');
                return
            end
        case 'mass'
            if mode==1      %constant-thrust
                dm = 0;
                for i=1:length(stage.engines)
                    if stage.engines(i).mode~=1
                        fprintf('vehicleTools couldnt calculate burned mass for the given stage. One of the engines is in profiled or unknown mode.\n');
                        return
                    end
                    dm = dm + stage.engines(i).flow;
                end
                result = input * dm;
            elseif mode==2  %constant-acceleration
                if length(stage.engines)~=1
                    fprintf('vehicleTools couldnt calculate burned mass for the given stage. Only single-engined stages can run in constant-acceleration mode.\n');
                    return
                end
                m0 = stage.m0;
                isp = stage.engines(1).isp0;
                alim = input(2);
                t = input(1);
                result = m0 - m0 * exp( -t / (isp/alim) );
            else
                fprintf('vehicleTools couldnt calculate burned mass for the given stage. Unknown throttle mode.\n');
                return
            end
        otherwise
            fprintf('vehicleTools failed to understand the task.\n');
            return
    end