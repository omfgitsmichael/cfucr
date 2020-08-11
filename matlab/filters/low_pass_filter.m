function [y_out, y_out_2] = low_pass_filter(y_in,y_int,alpha)

 y_out = alpha*y_in + (1-alpha)*y_int;
 y_out_2 = y_out;

end