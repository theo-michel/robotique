import matplotlib.pyplot as pyplot
import numpy as np

values = [ # Les valeurs à interpoler
    (0, 0),
    (1.2, 6),
    (2.5, -3),
    (3.3, -2),
    (4.2, -2),
    (5, 0)
]

def interpolate(values, t):
    """
    Interpolation linéaire entre plusieurs points
    """
    for i in range(len(values)-1):
        if (values[i][0]<t<values[i+1][0]):
            y1 = values[i][1]
            y2 = values[i+1][1]
            t1 = values[i][0]
            t2 = values[i+1][0]
            return  y1 + (t-t1)*(y2-y1)/(t2-t1)
            






ts = np.linspace(-1, 6, 100)
vs = [interpolate(values, t) for t in ts]

pyplot.scatter(ts, vs)
pyplot.show()