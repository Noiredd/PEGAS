%STS SRB thrust profile
%Eye-balled from https://en.wikipedia.org/wiki/Space_Shuttle_Solid_Rocket_Booster#Ignition
thrustProfile = [0   0.920;
                 10  0.987;
                 21  1.000;
                 27  0.918;
                 30  0.895;
                 34  0.852;
                 40  0.803;
                 42  0.787;
                 50  0.738;
                 52  0.738;
                 60  0.777;
                 63  0.787;
                 70  0.820;
                 77  0.830;
                 80  0.813;
                 84  0.787;
                 89  0.721;
                 95  0.689;
                 100 0.633;
                 104 0.590;
                 106 0.564;
                 110 0.541;
                 112 0.459;
                 115 0.262;
                 120 0.098;
                 124 0.033;
                 127 0.000];
if exist('scaleTo','var')
    scale_ = scaleTo / max(thrustProfile(:,1));
    for i=1:length(thrustProfile)
        thrustProfile(i,1) = thrustProfile(i,1) * scale_;
    end
end
clearvars scale_