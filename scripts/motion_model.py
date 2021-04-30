# set up an ackerman model to parse the Victoria park data
# implemented from:
# http://www-personal.acfr.usyd.edu.au/nebot/experimental_data/modeling_info/Ute_modeling_info.htm
import math

# magic numbers from vehicle specs
L=2.83
H=0.76
b=0.6
a=3.78

def ackerman_model(theta,v,dt):

    vc = v / (1 - math.tan(a) * H/L)

    dx = dt * vc * math.cos(theta) - vc / L * math.tan(theta) * (a*math.sin(theta) + b*math.cos(theta))
    dy = dt * vc * math.sin(theta) + vc / L * math.tan(theta) * (a*math.cos(theta) - b*math.sin(theta))
    dtheta = dt * vc/L*than(a)

    return dx, dy, dtheta
