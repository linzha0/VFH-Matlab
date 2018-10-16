function [ang_future ] =det_future_ang( Boundry,ang_A)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
Bound=Boundry;
ang_bound1=(2.5+5*(Bound(1)-1))*pi/180;
rad_x1=cos(ang_bound1)-cos(ang_A);
rad_y1=sin(ang_bound1)-sin(ang_A);
Point1=sqrt(rad_x1^2+rad_y1^2);



ang_bound2=(2.5+5*(Bound(2)-1))*pi/180;
rad_x2=cos(ang_bound2)-cos(ang_A);
rad_y2=sin(ang_bound2)-sin(ang_A);
Point2=sqrt(rad_x2^2+rad_y2^2);

if Point1 >=Point2
    gamma=(2.5+5*(Bound(2)-1))-60;
    if gamma <0
        gamma=gamma+360;
    end
    ang_future=gamma;
else
    gamma=(2.5+5*(Bound(1)-1))+60;
    if gamma <0
        gamma=gamma+360;
    else
        
    end
    ang_future=gamma;


end

