function slip=Sub_wheel_slip(v,w,CONST)

slip= (CONST.R * w - v)/abs(CONST.R * w);

end