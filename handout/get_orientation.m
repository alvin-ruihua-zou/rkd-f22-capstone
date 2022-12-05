function [angles] = get_orientation(R)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
psi = atan2(R(3,2),R(3,3));
phi = atan2(R(2,1),R(1,1));
theta = -asin(R(3,1));
angles = [psi;theta;phi];
end