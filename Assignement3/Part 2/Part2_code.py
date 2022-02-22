import numpy as np
import math

def to3D(vect, Min):
    new_vect = np.matmul(np.linalg.inv(Min), vect)
    return(new_vect)



def calculate_fov(v1, v2):
    #calculate norm
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)

    #calcualte dot product
    dot_product = np.dot(v1, v2)

    #calculate fov
    fov = math.acos(dot_product/(v1_norm*v2_norm))
    return(fov)

if(__name__ == '__main__'):

    Mex = [[1, 0, 0, 0],
           [0, 1, 0, 0],
           [0, 0, 1, 0]]

    Mex = np.array(Mex)

    Min = [[3691.05818495701, 0, 0],
           [0, 3685.98451263694, 0],
           [2288.06464667251, 1294.70233958207, 1]] #Instric Matrix from Matlab
    Min = np.array(Min).T

    img_size = np.array([4608, 2592]) #Width, Height, 
    
    #calculate pixels position
    P1 = np.array([img_size[0]/2, 0, 1])                    
    P2 = np.array([img_size[0]/2, img_size[1], 1])

    P3 = np.array([img_size[0], img_size[1]/2, 1])
    P4 = np.array([0, img_size[1]/2, 1])

    P5 = np.array([img_size[0], 0, 1])
    P6 = np.array([0, img_size[1], 1])

    #calculate pixel position in real
    v1 = to3D(P1, Min)
    v2 = to3D(P2, Min)

    v3 = to3D(P3, Min)
    v4 = to3D(P4, Min)

    v5 = to3D(P5, Min)
    v6 = to3D(P6, Min) 

    #calculate fov
    fov1 = calculate_fov(v1, v2)
    fov2 = calculate_fov(v3, v4)
    fov3 = calculate_fov(v5, v6)

    #put fov in degrees
    fov1_deg = np.round(math.degrees(fov1))
    fov2_deg = np.round(math.degrees(fov2))
    fov3_deg = np.round(math.degrees(fov3))

    print()
    print("Field of view in degree :")
    print("Vertically   :", fov1_deg)
    print("Horizontally :", fov2_deg)
    print("Diagonally   :", fov3_deg)
    print()

    #calculate the distance
    d = (10/2)/math.tan(fov1/2)
    print("distancce is :", d, "m")