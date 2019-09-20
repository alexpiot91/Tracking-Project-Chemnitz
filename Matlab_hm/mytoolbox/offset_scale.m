function mag_calib = offset_scale(mag_raw,x_avg,y_avg,z_avg,scalex_avg,scaley_avg,scalez_avg)
mag_calib = mag_raw;
mag_calib = mag_calib - [x_avg,y_avg,z_avg]; % offset
mag_calib = mag_calib.*[scalex_avg,scaley_avg,scalez_avg];


