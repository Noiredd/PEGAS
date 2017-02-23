function [density] = calculateAirDensity(pressure, temperature)
%d = CALCULATEAIRDENSITY(pressure, temp)
%Calculates air density from ideal gas law.
%
%INPUT
%    pressure   Air pressure in Pascals
%    temp       Air temperature in Kelvins
%
%OUTPUT
%    density    Air density in kg/m^2

    R = 286.9;  %specific gas constant for air [J/(kg*K)]
    density = pressure/(R*temperature);