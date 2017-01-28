function [result] = vehicleTools(task, stage, mode, input)
%   vehicleTools calculate often-used parameters for a given stage and mode
%   Input assumes different meanings based on the task given.
%   Task 'tgo' calculates maximum burn time for the stage, assuming all its
%   engines use the same fuel tanks; input = mass of fuel in stage.
%   Task 'mass' calculates mass of fuel burnt by all engines in the stage;
%   input = time of burn.
%   Mode can either be constant-thrust (1) or constant-acceleration (2) -
%   in this case, input must be an array containing in the first place the
%   value required by task, in the second an acceleration limit (in Gs).
%   Constant acceleration assumes vacuum conditions for Isp calculation.
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