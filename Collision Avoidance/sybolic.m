syms l r R
B = exp((l-r)/(l-R))
V = B/(1-B)
dVdl = diff(V, l)
dVdl = simplify(dVdl)
pretty(dVdl)



syms x y xa ya
l = sqrt((x-xa)^2+(y-ya)^2)
diff(l, x)