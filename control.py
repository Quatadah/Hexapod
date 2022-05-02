import math
import numpy as np
import interpolation 


# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alKashi theorem.
def alKashi(a, b, c, sign=-1):
    if a * b == 0:
        print("WARNING a or b is null in AlKashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a ** 2 + b ** 2 - c ** 2) / (2 * a * b)))) 

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

    return targets

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
    l1 = 45e-3
    l2 = 65e-3
    l3 = 87e-3
    xp = l1 + np.cos(beta)*l2 + np.cos(beta + gamma)*l3
    yp = np.sin(beta)*l2 + np.sin(beta + gamma)*l3
    
    x = np.cos(alpha) * xp 
    y = np.sin(alpha) * xp 
    z = yp
        
    return [x, y, z]

def trunc(x, a, b):
    return min(max(x, a), b)
def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle 
def trunc(x, a, b):
    return min(max(x, a), b)
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
    l0 = 45e-3
    l1 = 65e-3
    l2 = 87e-3
    dist_unit = 1
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit
    l1 = 45e-3
    l2 = 65e-3
    l3 = 87e-3
   
    Z_DIRECTION = 1
    sign=-1
    Theta1_SIGN=1
    Theta2_SIGN=1
    Theta3_SIGN=1
   

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
    # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)
    xp = math.sqrt(x * x + y * y) - l1

    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = -alKashi(l2, d, l3, sign=sign) + Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alKashi(l2, l3, d, sign=sign)
    #using radian for angles
    result=[theta1,theta2, theta3]
    #result=[Theta1_SIGN * modulopi(theta1), Theta2_SIGN * modulopi(theta2), Theta3_SIGN * modulopi(theta3)]
    print("les angles en radians sont:")
    print(result)
    print("********************************************")
    return result

#return the vector (x,y,z) with a teta rotation in the x,y plan
def rotation_2D(theta, x):
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])    
    return np.dot(rot, x)
"""
position = [0,0]
#leg1,3
triPos1 = [0,0]
#leg2,4
triPos2 = [0,0]
def update(teta):
#Update Position according to each leg 
    global position, triPos1, triPos2
    length=0.016
    height=0.016
    if tri1[1] == 0:
        triPos2 = [length / 2, 0, 0]
        previousTri = triPos1
        triPos1 = tri1
        position[0] -= (triPos1[0] - previousTri[0]) * np.cos(teta)
        position[1] -= (triPos1[0] - previousTri[0]) * np.sin(teta)
    else:
        triPos1 = [length / 2, 0, 0]
        previousTri = triPos2
        triPos2 = tri2
        position[0] -= (triPos2[0] - previousTri[0]) * np.cos(teta)
        position[1] -= (triPos2[0] - previousTri[0]) * np.sin(teta)
        """
def draw(t):
    """
    python simulator.py -m draw

    Le robot est figé en l'air, on ne contrôle qu'une patte

    Le but est, à partir du temps donné, de suivre une trajectoire de triangle. Pour ce faire, on
    utilisera une interpolation linéaire entre trois points, et la fonction inverse précédente.

    - Entrée: t, le temps (secondes écoulées depuis le début)
    - Sortie: un tableau contenant les 3 positions angulaires cibles (en radians)
    """
    freq=2
    T = 1 / freq
    triangle = interpolation.LinearSpline3D()
    triangle.add_entry(0,0.04, 0.04,  0)
    triangle.add_entry(T/3, 0.04,0.15, 0)
    triangle.add_entry(2 * T / 3, 0.15,0.04,0)
    triangle.add_entry(T,0.04,0.04, 0)
    tri1 = triangle.interpolate(t % T)
    #tri2 = triangle.interpolate((t + T/3) % T) 
    """ triangle.add_entry(0,0, 0, 0)
    triangle.add_entry(1,2, 0, 0)
    triangle.add_entry(2,0, 2, 0)
    l=triangle.interpolate(t)
    """
    #update(5*np.pi/4)
    leg1=[tri1[0], tri1[1],0]
    #update(3*np.pi/4)
    #leg2=[tri2[0], tri2[1],0]
    #update(np.pi/4)
    #leg3=[tri1[0], tri1[1],0]
    #update(-np.pi/4)
    #leg4=[tri2[0], tri2[1],0]
    #legs(leg1, leg2, leg3, leg4)
    """print(leg1)
    print("/*************/")
    print(leg2)
    print("/*************/")
    print(leg3)
    print("/*************/")"""
    return inverse(leg1[0], leg1[1], leg1[2])
   

def legs(leg1, leg2, leg3, leg4):
    """
    python simulator.py -m legs

    Le robot est figé en l'air, on contrôle toute les pattes

    - Sliders: les 12 coordonnées (x, y, z) du bout des 4 pattes
    - Entrée: des positions cibles (tuples (x, y, z)) pour le bout des 4 pattes
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """ 
    d = 40e-3
    targets = [0]*12

    leg4p = [0] * 3
    target = rotation_2D(-np.pi/4, [leg4[0], leg4[1]])
    leg4p[0] = target[0] - d
    leg4p[1] = target[1] 
    leg4p[2] = leg4[2]
    inverse4 = inverse(leg4p[0], leg4p[1], leg4p[2]) 
    target = rotation_2D(np.pi/4, [leg3[0], leg3[1]])
    leg4p[0] = target[0] - d
    leg4p[1] = target[1] 
    leg4p[2] = leg3[2]
    inverse3 = inverse(leg4p[0], leg4p[1], leg4[2]) 
    target = rotation_2D(3*np.pi/4, [leg2[0], leg2[1]])
    leg4p[0] = target[0] - d
    leg4p[1] = target[1] 
    leg4p[2] = leg2[2]
    inverse2 = inverse(leg4p[0], leg4p[1], leg4p[2]) 
    target = rotation_2D(5*np.pi/4, [leg1[0], leg1[1]])
    leg4p[0] = target[0] - d
    leg4p[1] = target[1] 
    leg4p[2] = leg1[2]
    inverse1 = inverse(leg4p[0], leg4p[1], leg4p[2])   
  
    
    return inverse1 + inverse2 + inverse3 + inverse4
def walk(t, speed_x, speed_y, speed_rotation):
    """
    python simulator.py -m walk

    Le but est d'intégrer tout ce que nous avons vu ici pour faire marcher le robot

    - Sliders: speed_x, speed_y, speed_rotation, la vitesse cible du robot
    - Entrée: t, le temps (secondes écoulées depuis le début)
            speed_x, speed_y, et speed_rotation, vitesses cibles contrôlées par les sliders
    - Sortie: un tableau contenant les 12 positions angulaires cibles (radian) pour les moteurs
    """
  
    """leg1=[45e-3*np.cos(speed_rotation*t+5*np.pi/4)+speed_x,-45e-3*np.cos(speed_rotation*t+5*np.pi/4)+speed_y,0]
    leg2=[45e-3*np.cos(speed_rotation*t+3*np.pi/4)+speed_x,-45e-3*np.cos(speed_rotation*t+3*np.pi/4)+speed_y,0]
    leg3=[45e-3*np.cos(speed_rotation*t+2*np.pi/4)+speed_x,-45e-3*np.cos(speed_rotation*t+2*np.pi/4)+speed_y,0]
    leg4=[45e-3*np.cos(speed_rotation*t+1*np.pi/4)+speed_x,-45e-3*np.cos(speed_rotation*t+1*np.pi/4)+speed_y,0]
    targets =leg1+leg2+leg3+leg4 """
    leg3=[0,0,-0.14]
    leg4=[0,0,-0.14]
    freq=2
    T = 1 / freq
    triangle = interpolation.LinearSpline3D()
    triangle.add_entry(0,0.04, 0.15,  -0.14)
    triangle.add_entry(T/3, 0.07,0.15, -0.14)
    triangle.add_entry(2 * T / 3, 0.10,0.15,-0.14)
    triangle.add_entry(T,0.04,0.15, -0.14)
    tri1 = triangle.interpolate(t % T)
    tri2 = triangle.interpolate((t+T/3) % T) 
    tri3= triangle.interpolate((t+2*T/3) % T)
    tri4= triangle.interpolate((t+T)  % T)
    """ triangle.add_entry(0,0.08, 0.05,  -0.14)
    x1,y1= rotation_2D(np.pi/5, [0.08, 0.05] )
    x2,y2= rotation_2D(2*np.pi/5, [0.08, 0.05] )
    x3,y3= rotation_2D(3*np.pi/5, [0.08, 0.05] )
    x4,y4= rotation_2D(4*np.pi/5, [0.08, 0.05] )
    x5,y5= rotation_2D(5*np.pi/5, [0.08, 0.05] )
     
    triangle.add_entry(T/10, x1,y1, -0.04)
    triangle.add_entry(2*T/10, x2,y2, -0.04)
    triangle.add_entry(3*T/10, x3,y3, -0.04)
    triangle.add_entry(4 * T / 10, x4,y4,-0.04)
    triangle.add_entry(5*T/10, x5,y5, -0.04)
    triangle.add_entry(6*T/10, x5,y5, -0.04)
    triangle.add_entry(7*T/10, x4,y4, -0.04)
    triangle.add_entry(8 * T / 10, x3,y3,-0.04)
    triangle.add_entry(9 * T / 10, x2,y2,-0.04)
    triangle.add_entry(T,x1,y1, -0.04)
    tri1 = triangle.interpolate(t % T)
    tri2 = triangle.interpolate(t% T) 
    tri3= triangle.interpolate(t % T)
    tri4= triangle.interpolate(t % T)
   
    #update(5*np.pi/4) 
    """
    leg1=[tri1[0], tri1[1], tri1[2]]

    #update(3*np.pi/4)
    leg2=[tri2[0], tri2[1], tri2[2]]
    #update(np.pi/4)
    leg3=[tri3[0], tri3[1], tri3[2]]
    #update(-np.pi/4)
    leg4=[tri4[0], tri4[1],tri4[2]]
    #legs(leg1, leg2, leg3, leg4)
    """print(leg1)
    print("/*************/")
    print(leg2)
    print("/*************/")
    print(leg3)
    print("/*************/")"""
    return inverse(leg1[0], leg1[1], leg1[2])+inverse(leg2[0], leg2[1], leg2[2])+inverse(leg3[0], leg3[1], leg3[2])+inverse(leg4[0], leg4[1], leg4[2])

if __name__ == "__main__":
    print("N'exécutez pas ce fichier, mais simulator.py")