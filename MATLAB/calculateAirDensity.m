%calculateAirDensity.m
%Inputs pressure in [Pa] and temperature in [K], outputs gas density from ideal gas law.
function [density] = calculateAirDensity(pressure, temperature)
    R = 286.9;  %individual gas constant [J/(kg*K)]
    density = pressure/(R*temperature);