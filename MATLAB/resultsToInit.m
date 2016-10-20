%resultsToInit.m
%Converts flightSim3D results into input ininitialization struct.
function [init] = resultsToInit(results)
    n = length(results.Plots.t);
    init = struct('type', 1,...
                  't', results.Plots.t(n),...
                  'r', results.Plots.r(n,:),...
                  'v', results.Plots.v(n,:));
    %Handle UPFG state passing. Copy final state from results, create a
    %dummy state otherwise.
    if isfield(results, 'upfg')
        init().UPFG = results.upfg;
    else
        init().UPFG = struct();
    end;
end