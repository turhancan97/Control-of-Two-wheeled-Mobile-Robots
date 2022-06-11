function [out] = dV(inp)

global R_;
global r_;
delta_ = 0.001;
R = R_;
r = r_;
l = inp;
if (l < r)
    out = 0;
elseif (l > R-delta_) 
    out = 0;
else    
    out = -(R-r)*exp((l-r)/(l-R))/(-1+exp((l-r)/(l-R)))^2/(l-R)^2;
end;  
