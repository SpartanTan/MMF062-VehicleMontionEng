function slip=Sub_wheel_slip(v,w,CONST)
R = CONST.R;
slip=  (R*w-v)/abs(R*w);

end