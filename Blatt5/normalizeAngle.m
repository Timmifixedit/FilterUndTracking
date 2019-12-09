function phi = normalizeAngle(phi_in)
% correct the angular value difference if one and only one value is < 0:
% the angular measurement range is (-180:180], so the difference between
% the values are different than the numeric result!!

phi = abs(phi_in);
% normalize phi to value in interval [-pi;pi]
phi = mod(phi, 2*pi);
if(phi > pi)
  phi = phi - 2*pi;
end;
phi = phi*sign(phi_in);