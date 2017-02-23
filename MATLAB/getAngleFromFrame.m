function [angle] = getAngleFromFrame(vector, frame, type)
%angle = GETANGLEFROMFRAME(vector, frame, type)
%Calculates pitch or yaw angle of a vector in a given reference frame.
%
%INPUT
%    vector     XYZ vector
%    frame      3x3 matrix of unit basis vectors given row-wise in the
%               following order:
%                   local "up" (direction of zero pitch)
%                   local "north" (direction of 90 degree yaw)
%                   local "east" (direction of zero yaw)
%    type       string, either 'pitch' or 'yaw'
%
%OUTPUT
%   angle       requested angle in degrees

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