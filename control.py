from cmath import acos
import math
import numpy as np
from interpolation import LinearSpline
from interpolation import LinearSpline3D

def sandbox(t):
    """
    python simulator.py -m sandbox

    Un premier bac à sable pour faire des expériences

    La fonction reçoit le temps écoulé depuis le début (t) et retourne une position cible
    pour les angles des 12 moteurs

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    # Par exemple, on envoie un mouvement sinusoidal
    targets = [0]*12
    targets[0] = np.sin(t)
    targets[1] = np.sin(t)
    targets[2] = np.sin(t)
    targets[3] = np.sin(t)
    targets[4] = np.sin(t)
    targets[5] = np.sin(t)
    targets[6] = np.sin(t)
    targets[7] = np.sin(t)
    targets[8] = np.sin(t)
    targets[9] = np.sin(t)
    targets[10] = np.sin(t)
    targets[11] = np.sin(t)


    return targets
# Dimensions (mm)
l0, l1, l2 = 0.041, 0.065, 0.087

def forward_kinematics(alpha, beta, gamma):
    """Prend en entrée les angles moteurs et produit la position atteinte"""
    xp = l0 + math.cos(beta)*l1 + math.cos(beta + gamma)*l2
    yp = math.sin(beta)*l1 + math.sin(beta + gamma)*l2
    
    x = math.cos(alpha) * xp 
    y = math.sin(alpha) * xp 
    z = yp
    
    return [x, y, z]

def direct(alpha, beta, gamma):
    """
    python simulator.py -m direct

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument la cible (alpha, beta, gamma) des degrés de liberté de la patte, et produit
    la position (x, y, z) atteinte par le bout de la patte

    - Sliders: les angles des trois moteurs (alpha, beta, gamma)
    - Entrées: alpha, beta, gamma, la cible (radians) des moteurs
    - Sortie: un tableau contenant la position atteinte par le bout de la patte (en mètres)
    """

    return forward_kinematics(alpha,beta,gamma)
def normalise(x):
    if ( x > 1): 
        x = 1
    elif x < -1:
        x = -1 
    return x
def inverse(x, y, z):
    """
    python simulator.py -m inverse

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Reçoit en argument une position cible (x, y, z) pour le bout de la patte, et produit les angles
    (alpha, beta, gamma) pour que la patte atteigne cet objectif

    - Sliders: la position cible x, y, z du bout de la patte
    - Entrée: x, y, z, une position cible dans le repère de la patte (mètres), provenant du slider
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """

    a=-z/(math.sqrt( (math.sqrt(x**2 + y**2) - l0)**2 + z**2  ))
    b=(l1**2 - l2**2 + ( math.sqrt(x**2 + y**2) - l0 )**2 + z**2 )/(2*l1*(math.sqrt( (  math.sqrt(x**2 +y**2) - l0 )**2  + z**2 ))) 
    c=( l1**2 + l2**2 - ( math.sqrt(x**2 + y**2) - l0 )**2 -(z**2)) / (2*l1*l2)
    a=normalise(a)
    b=normalise(b)
    c=normalise(c)
    teta0 = math.atan2(y,x)
    teta1 = math.acos(a) + math.acos(  b ) - math.pi/2
    teta2 = math.pi - math.acos( c   )
    return [teta0,teta1, teta2]


spline = LinearSpline3D()
spline.add_entry(0, 0.14, -0.05, 0)
spline.add_entry(2, 0.14, 0.0, 0.05)
spline.add_entry(4, 0.14, 0.05, 0)
spline.add_entry(6, 0.14, -0.05, 0)

def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    print(t)
    r = spline.interpolate(t % 6)

    return inverse(r[0],r[1],r[2])
    

def legs(leg1, leg2, leg3, leg4):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """

    targets = [0]*12

    return targets

def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
    targets = [0]*12

    return targets

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")