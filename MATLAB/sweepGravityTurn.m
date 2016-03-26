%sweepGravityTurn.m
%Performs a natural gravity turn (with prograde lock) simulation for a
%given set of parameters (initial velocities and pitch angles). 'Sweeps' a
%2-D parameter space generating a result vector for each point (apoapsis
%altitude and maxQ value).
%Dependencies:
%   flightSim2D.m       legacy 2D simulation, because faster than 3D
%                       consider complete migration to 3D for consistency
function [a, q] = sweepGravityTurn(vehicle, conditions, velocities, pitches, precision)
    a = zeros(length(velocities),length(pitches));
    q = zeros(length(velocities),length(pitches));
    for i = 1:length(velocities)
        for j = 1:length(pitches)
            c=struct('type',0, 'p', pitches(j), 'v', velocities(i));
            r=flightSim2D(vehicle, conditions, c, precision);
            a(i,j) = r.Apoapsis;
            q(i,j) = r.maxQv;
        end;
    end;
    a=a.';
    q=q.';
end