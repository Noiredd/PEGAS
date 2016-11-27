%getAngleFromFrame.m
%Calculates pitch or yaw angle (depending on type) for a given vector in a
%given reference frame (consisting of UP, NORTH, and EAST unit vectors in
%rows of a 3x3 matrix). Returns degrees.
function [angle] = getAngleFromFrame(vector, frame, type)
    vector = unit(vector);
    if strcmp(type,'pitch')
        angle = acosd(dot(vector, frame(1,:)));
    elseif strcmp(type,'yaw')
        angle = acosd(dot(vector, frame(3,:)));
        %correct for direction of the angle
        if dot(frame(1,:), cross(vector, frame(3,:))) > 0
            angle = -angle;
        end;
    else
        disp('Unknown parameter (getAngleFromFrame).\n');
        angle = 0;
    end
end