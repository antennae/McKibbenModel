import array
import numpy as np
from splib3.topology import remeshing as rf
from stl import mesh
from math import sin,cos, sqrt, acos, radians, dist, ceil
import math
import time

def obtenir_autres_axes(axis):
    # Définit les combinaisons possibles pour chaque valeur de `axis`
    correspondance = {
        0: (1, 2),
        1: (0, 2),
        2: (0, 1)
    }
    
    # Retourne la paire correspondante
    return correspondance.get(axis, None)  # Retourne None si `axis` est invalide


def conv_tab_from_ind_tab(ind_tab): # a mettre dans Remeshing_functions.py
    conv_tab = []
    for i in range(len(ind_tab)):
        # conv_tab.append([ind_tab[i],i])
        conv_tab.append([i,ind_tab[i]])
    return conv_tab

def calculer_centre(points):
    if not points:
        return None  # Retourne None si la liste de points est vide

    # Calcul de la moyenne des coordonnées x et y
    somme_x = sum(x for x, y, z in points)
    somme_y = sum(y for x, y, z in points)
    somme_z = sum(z for x, y, z in points)
    n = len(points)
    
    centre_x = somme_x / n
    centre_y = somme_y / n
    centre_z = somme_z / n
    
    return (centre_x, centre_y,centre_z)

def trier_points_horaire_avec_indices(points, indices, axis = 2, centre=None):

    if centre == None :
        centre = calculer_centre(points)

    [axis_0,axis_1] = obtenir_autres_axes(axis) # Les deux axes qui vont définir le plan dans lequel va se calculer le sens horaire

    # Fonction pour calculer l'angle entre le point et le centre du cercle
    def angle(point):
        value_0 = point[1][axis_0]  # Utiliser les coordonnées du point (non l'indice)
        value_1 = point[1][axis_1]  
        c_0 = centre[axis_0]
        c_1 = centre[axis_1]
        return math.atan2(value_1 - c_1, value_0 - c_0)
    
    # Associer chaque point avec son indice fourni
    points_avec_indices = list(zip(indices, points))
    
    # Trier les points en fonction de l'angle dans le sens horaire (angle décroissant)
    points_tries = sorted(points_avec_indices, key=angle, reverse=True)
    
    # Extraire les indices et les points triés
    indices_tries = [i for i, _ in points_tries]
    points_triés = [p for _, p in points_tries]
    
    return points_triés, indices_tries

def AddConstrainCircles(parent,circle_tab,circle_ind_tab,conv_tab,axis,stiffness = 10000): # A mettre dans SoftRobot ? # Ajouter la raideur du ressort en paramètre ?
    """
    Fonction qui ajoute les ressorts autour des cavités pour éviter les déformations latérales
    """
    # print(circle_ind_tab)
    circle_tab_old = rf.new_idx_from_conv_tab(mesh= circle_ind_tab,conv_tab=conv_tab) # pour remettre les anciens indices, et ainsi correspondre aux noeuds du maillage complet
    ind = 0
    for u in range(len(circle_ind_tab)):
        ind = ind + 1
        cercle = circle_tab[u]
        cercle_ind = circle_ind_tab[u]
        cercle_ind_old = circle_tab_old[u]
        [new_circle_pt,new_ind_tab] = trier_points_horaire_avec_indices(points = cercle, indices = cercle_ind, axis = axis)#,ind_tab = cercle_ind_old) # pour récupérer les indices triés dans le sens horaire
        print("cercle trié")
        print(new_ind_tab)
        if len(cercle) > 2 : #on ne place les ressorts que si suffisamment de points sont alignés
            for ind_cercle in range(len(cercle)):
                ind_0 = ind_cercle
                ind_1 = ind_cercle + 1 
                if ind_1 > len(cercle)-1 :
                    ind_1 = 0
                p1 = new_circle_pt[ind_0]
                p2 = new_circle_pt[ind_1]
                d = [dist(p1,p2)] # on suppose ici que tous les points d'un cercle de la cavité sont espacés de la même distance => refaire un code qui place les ressorts 1 à 1 pour être utilisable pour toutes les géométries ?
                NoeudCercle = parent.addChild("Ressort" + str(ind_cercle) + "_" +  str(u))
                # new_ind_tab_2 = rf.shift_tab(tab= new_ind_tab) # tableau des indices décalés d'un point, pour relier chaque point du cercle au point suivant
                NoeudCercle.addObject("MeshSpringForceField", name="Springs" ,stiffness= stiffness,indices1 =new_ind_tab[ind_0], indices2 = new_ind_tab[ind_1] ,length = d)# damping="4"

def ConstrainCavity(points,parent,indices=None,axis = 0,tolerance = 0,spring_stiffness=10000): # A mettre dans SPLIB ?
    """
    parent = noeud SOFA sur lequel on va mettre les resorts 
    axis = axe selon lequel on va placer les cercles de ressorts successifs
    tolerance = distance (dépend du maillage, généralement en mm) maximale selon laquelle on va considérer deux points comme étant à une position simimlaire selon l'axe axis, et qui si ils sont assez proches formeront un deux points successifs de la ligne de ressort.


    Fonction qui va trier les points et le maillage passé en argument afin d'ajouter des ressorts pour contraindre la cavité et renvoyer les points, 
    le maillage et le tableau de conversion pour créer le noeud qui contient la cavité.
    """
    [circles, ind_tab] = rf.circle_detection_axis(points = points, axis = axis, tolerance = tolerance,indices = indices) # circles position good # detecte les positions des cercles le long des cavités
    conv_tab = conv_tab_from_ind_tab(rf.default_indices(len(points)))
    AddConstrainCircles(parent=parent,circle_tab = circles,circle_ind_tab=ind_tab,conv_tab = conv_tab,axis = axis,stiffness=spring_stiffness) # Pour créer les ressorts qui constraigenent les déformations latérales 

    # return [new_points, triangles,conv_tab]