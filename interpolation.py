import numpy as np


class LinearSpline:
    def __init__(self):
        self.values = []

    def add_entry(self, t, x):
        self.values.append((t, x))

    def interpolate(self, t):
        """
        Interpolation linéaire entre deux points
        """
        values = self.values
        for i in range(len(values)-1):
            if (values[i][0]<t<=values[i+1][0]):
                y1 = values[i][1]
                y2 = values[i+1][1]
                t1 = values[i][0]
                t2 = values[i+1][0]
                return  y1 + (t-t1)*(y2-y1)/(t2-t1)
                



class LinearSpline3D:
    def __init__(self):
        self.values = []

    def add_entry(self, t, x, y ,z):
        self.values.append((t, x, y, z))

    def interpolate(self, t):
        """
        Linear interpolation between two points in 3D space
        """
        values = self.values
        for i in range(len(values)-1):
            if (values[i][0]<t<=values[i+1][0]):
                x1 = values[i][1]
                x2 = values[i+1][1]
                y1 = values[i][2]
                y2 = values[i+1][2]
                z1 = values[i][3]
                z2 = values[i+1][3]
                t1 = values[i][0]
                t2 = values[i+1][0]
                return  x1 + (t-t1)*(x2-x1)/(t2-t1), y1 + (t-t1)*(y2-y1)/(t2-t1), z1 + (t-t1)*(z2-z1)/(t2-t1)
        if t>values[-1][0]:
            return values[-1][1], values[-1][2], values[-1][3]
        elif t<values[0][0]:
            return values[0][1], values[0][2], values[0][3]
        else:
            return 0,0,0
                
        
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    spline = LinearSpline3D()
    spline.add_entry(0., 0., 1., 2.)
    spline.add_entry(0.5, 0.2, 3., -2.)
    spline.add_entry(1.5, 0., 5., -2.)
    spline.add_entry(2.3, 0.6, 0., 0.1)

    ts = np.arange(-0.1, 2.5, 0.01)
    xyzs = np.array([spline.interpolate(t) for t in ts])

    plt.scatter(ts, xyzs.T[0])
    plt.scatter(ts, xyzs.T[1])
    plt.scatter(ts, xyzs.T[2])
    plt.show()
