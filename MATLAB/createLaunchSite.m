function [ls] = createLaunchSite(name)
%site = CREATELAUNCHSITE(name)
%Outputs a properly formatted init struct from a given string representing
%a launch site name.
%
%INPUT
%    name       Short name of a launch site. Supported choices:
%                full name                           short name
%               ------------------------------------------------
%               Kennedy Space Center                KSC, Kennedy
%               Vandenberg Air Force Base           Vandenberg
%               Guiana Space Centre                 Kourou
%               Baikonur Cosmodrome                 Baikonur
%               Plesetsk Cosmodrome                 Plesetsk
%               Uchinoura Space Center              Uchinoura
%               RAAF Woomera Test Range             Woomera
%               Jiuquan Satellite Launch Center     Jiuquan
%
%OUTPUT
%    site       Init struct of type 0 containing longitude, latitude and
%               altitude of a selected launch site.
    switch name
        case 'KSC'
            ls = struct('type', 0, 'lat', 28.52406, 'lon', -80.65085, 'alt', 0);
        case 'Kennedy'
            ls = createLaunchSite('KSC');
        case 'Vandenberg'
            ls = struct('type', 0, 'lat', 34.75083, 'lon', -120.49778, 'alt', 0);
        case 'Kourou'
            ls = struct('type', 0, 'lat', 5.15972, 'lon', -52.65028, 'alt', 0);
        case 'Baikonur'
            ls = struct('type', 0, 'lat', 45.91194, 'lon', 63.31028, 'alt', 92);
        case 'Plesetsk'
            ls = struct('type', 0, 'lat', 62.92556, 'lon', 40.57778, 'alt', 131);
        case 'Uchinoura'
            ls = struct('type', 0, 'lat', 31.25194, 'lon', 131.08914, 'alt', 0);
        case 'Woomera'
            ls = struct('type', 0, 'lat', -30.94907, 'lon', 136.53418, 'alt', 137);
        case 'Jiuquan'
            ls = struct('type', 0, 'lat', 41.11803, 'lon', 100.46330, 'alt', 1045);
        otherwise
            ls = createLaunchSite('KSC');
            fprintf('Unknown launch site - returning KSC.');
    end