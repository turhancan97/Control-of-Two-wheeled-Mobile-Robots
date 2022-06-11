function [out] = dVxy(inp)
pxy = inp(1:2);
oxy = inp(3:4);

distvec = pxy-oxy;
l = norm(distvec);
B = dV(l);

if l > 0,
    dir = distvec/l;
    grad = B * dir;
else
    grad = [0 0];
end

out = grad;

