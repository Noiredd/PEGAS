%getAngleFromFrame.m
%Calculates pitch or yaw angle (depending on type) for a given vector in a
%given reference frame (consisting of UP, NORTH, and EAST unit vectors in
%rows of a 3x3 matrix). Returns degrees.
function [angle] = getAngleFromFrame(vector, frame, type)
    vector = unit(vector);
    if strcmp(type, 'pitch')
        angle = safeAcosd(dot(vector, frame(1,:)));
    elseif strcmp(type, 'yaw')
        inplane = vector - frame(1,:)*dot(vector, frame(1,:));
        inplane = unit(inplane);
        angle = safeAcosd(dot(inplane, frame(3,:)));
        %correct for direction of the angle
        tangential = cross(frame(1,:), frame(3,:));
        if dot(inplane, tangential) < 0
            angle = -angle;
        end;
    else
        disp('Unknown parameter (getAngleFromFrame).\n');
        angle = 0;
    end
    if abs(imag(angle))>0
        disp('-');
    end
end

function [a] = safeAcosd(angle)
    angle = min(angle, 1);
    angle = max(angle,-1);
    a = acosd(angle);
end