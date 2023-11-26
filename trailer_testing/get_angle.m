function angle = get_angle(M)
angle = ones(3,1);
angle(2) = -asin(M(3,1));
theta = angle(2);
angle(3) = atan2(M(3,2)/cos(theta),M(3,3)/cos(theta) );
angle(1) = atan2(M(2,1)/cos(theta),M(1,1)/cos(theta));
angle = angle/pi*180;

end