from math import pi
import numpy as np

class Pose:
    """2D pose"""

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def calc_spline_course(sp,ds):
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))
    return rx, ry, ryaw, rk, s

def generate_formation(geometry,d,Tg,Tm,c):

    if geometry == 'line':
        formation =  np.array([[0, d+(d/2)],
                                [0, +d/2], 
                                [0, -d/2], 
                                [0,-d-(d/2)]]) # IN LINEA  
        mean = [Tg+6*d/c,Tg+4*d/c,Tg+2*d/c,0]  #medium latencies between each AUV and the 4th (in fact latencies 0.0 for the 4th).
        variance = [Tm*3.0, Tm*1.8, Tm*0.8, 0.0] #the same as before vor the variances.

    elif geometry == 'line2':
        formation =  np.array([ [0, +d/2],
                                [0, -d/2],
                                [0, +d/2],
                                [0, -d/2]])
        mean = [Tg+6*d/c,Tg+4*d/c,0,0]  #medium latencies between each AUV and the 4th (in fact latencies 0.0 for the 4th).
        variance = [Tm*1.0, Tm*0.8, Tm*0.3, 0.0] #the same as before vor the variances.

    elif geometry == 'column':
        formation = [d*3,d*2,d*1,0] # IN COLONNA
        mean = [Tg+6*d/c,Tg+4*d/c,Tg+2*d/c,0]  
        variance = [Tm*3.0, Tm*1.8, Tm*0.8, 0.0]
        '''formation =  np.array([[0,0],
                                [d,0], 
                                [2*d,0], 
                                [3*d,0]]) # IN LINEA  
        mean = [Tg+6*d/c,Tg+4*d/c,Tg+2*d/c,0]  #medium latencies between each AUV and the 4th (in fact latencies 0.0 for the 4th).
        variance = [Tm*3.0, Tm*1.8, Tm*0.8, 0.0] #the same as before vor the variances.'''

    elif geometry == 'column2':
        formation =  [d,d/2,d,d/2]
        mean = [Tg*d/c,Tg*d/c,Tg*d/c,0]  
        variance = [Tm*1.5, Tm*0.8, Tm*0.3, 0.0]

    elif geometry == 'one_auv':
        formation =  np.array([[0, 0]]) # TRAPEZOIDALE 

    return formation, mean, variance