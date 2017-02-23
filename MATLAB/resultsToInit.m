function [init] = resultsToInit(results)
%init = RESULTSTOINIT(results)
%Converts a results struct into an initialization struct, allowing easily
%continuing a flight.
%
%INPUT
%    results    Results struct as returned by flightSim3D.
%
%OUTPUT
%    init       Init struct of type 1.

    n = length(results.Plots.t);
    init = struct('type', 1,...
                  't', results.Plots.t(n),...
                  'r', results.Plots.r(n,:),...
                  'v', results.Plots.v(n,:));
    %Handle UPFG state passing. Copy final state from results, create a
    %dummy state otherwise.
    if isfield(results, 'UPFG')
        init().upfg = results.UPFG;
    else
        init().upfg = struct();
    end;
end