def make_rel_path(path):
    ys = [p[0] for p in path]
    xs = [p[1] for p in path]
    yr = map(lambda a1,a0 : a1-a0, ys[1:], ys[:-1])
    xr = map(lambda a1,a0 : a1-a0, xs[1:], xs[:-1])
    return zip(yr, xr)

def compress_path(path):
    ret = []
    i = 0
    while i < len(path):
        j = i + 1
        while (j<len(path) and path[j] == path[i]):
            j = j + 1
            pass
        d = j-i
        (y, x) = path[i]
        ret.append((y*d,x*d))
        i=j
        pass
    return ret

def make_abs_path(rpath):
    path = []
    pos = (0,0)
    for i in range(0, len(path0)):
        (y0,x0) = pos
        (y1,x1) = rpath[i]
        pos = [y0+y1,x0+x1]
        path.append(pos)
        pass
    return path

def find_path(map, x0, y0, x1, y1):
    path0 = [[1,0]]*2+\
            [[0,1]]+\
            [[1,0]]*5+\
            [[0,1]]*18+\
            [[1,0]]*18+\
            [[0,-1]]*17+\
            [[-1,0]]*6+\
            [[0,1]]*8
    path = make_abs_path(path0)
    print(path)
    rpath = make_rel_path(path)
    print(rpath)
    cpath = compress_path(rpath)
    print(cpath)
    return cpath
    
