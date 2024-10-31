"""
Auteur : Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2024
Propriétaire : Université de Lille - CNRS 
License : Non définie, mais développé dans une démarche Open-Source et Logiciel Libre avec volonté de partage et de travail collaboratif. Développé dans un but non-marchand, en cas d'utilisation commerciale, merci de minimiser les prix et de favoriser le partage gratuit de tout ce qui peut l'être. A utiliser dans des buts prenant en compte les questions éthiques et morales (si possible non-militaire, ne rentrant pas dans le cadre de compétition, de monopole, ou de favorisation d'interets privés).
"""

 # coding=utf-8

import Sofa
import array
import numpy as np
from splib3.topology import remeshing as rf
from stl import mesh
from math import sin,cos, sqrt, acos, radians, dist, ceil
import math
import time

def createCavity(parent,name_c,i,cavity_model,act_flag): # for v1 -------

    """
    name__c : name of the created node
    i : cavity number
    cavity_model : cavity model filename (should be .stl)
    act_flag = 0 : Inverse Control
    act_flag = 1 : Direct Control

    """
    bellowNode = parent.addChild(name_c+str(i+1))
    MeshLoad = bellowNode.addObject('MeshSTLLoader', filename=cavity_model, flipNormals='0', triangulate='true', name='meshLoader',rotation=[0,0,0], translation=[0, 0,0])#, rotation=[self.ang_dec*self.i_cavity,0,0] if pre-rotated 3D model
    MeshLoad.init()
    points  = MeshLoad.position.value
    triangles = MeshLoad.triangles.value
    bellowNode.addObject('MeshTopology', src='@meshLoader', name='Cavity')
    bellowNode.addObject('MechanicalObject', name='chambreA'+str(i+1),rotation=[0, 0 , 0])#,translation = [0,0,h_module*i]) # 90 on y
    bellowNode.addObject('TriangleCollisionModel', moving='0', simulated='1')
    bellowNode.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.49,  youngModulus=100, thickness = 5) # stable youngModulus = 500 / réel ? = 103
    bellowNode.addObject('UniformMass', totalMass=1000, rayleighMass = 0)

    if act_flag == 0 :
        bellowNode.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh'+str(i+1)+'.triangles',minPressure = 0,maxPressure = 300)#,maxPressureVariation = 20)#,valueType=self.value_type)
    elif  act_flag == 1 :
        bellowNode.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMesh'+str(i+1)+'.triangles', value=50,minPressure = 0,maxPressure = 300, valueType="pressure" )#,maxPressureVariation = 20)#,

    bellowNode.init()
    BaseBox = bellowNode.addObject('BoxROI', name='boxROI_base', box=[-8, -8, -1, 8, 8, 1], drawBoxes=True, strict=False,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
    BaseBox.init()
    print("selected : ")
    print(BaseBox.indices.value)
    bellowNode.addObject('RestShapeSpringsForceField', points=BaseBox.indices.value, angularStiffness=1e5, stiffness=1e5) # pour accrocher la base du robot dans l'espace

    return bellowNode

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
    Fonction qui va trier les points et le maillage passé en argument afin d'ajouter des ressorts pour contraindre la cavité et renvoyer les points, 
    le maillage et le tableau de conversion pour créer le noeud qui contient la cavité.
    """
    [circles, ind_tab] = rf.circle_detection_axis(points = points, axis = axis, tolerance = tolerance,indices = indices) # circles position good # detecte les positions des cercles le long des cavités
    conv_tab = conv_tab_from_ind_tab(rf.default_indices(len(points)))
    AddConstrainCircles(parent=parent,circle_tab = circles,circle_ind_tab=ind_tab,conv_tab = conv_tab,axis = axis,stiffness=spring_stiffness) # Pour créer les ressorts qui constraigenent les déformations latérales 

    # return [new_points, triangles,conv_tab]


def createScene(rootNode):

    # rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/SoftRobots/lib/') #libSoftRobots.so 1.0
    # rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/ModelOrderReduction/lib/') #libSoftRobots.so 1.0
    # rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/BeamAdapter/lib')#/libBeamAdapter.so 1.0

    # required plugins:
    pluginNode  = rootNode.addChild('pluginNode')
    pluginNode.addObject('RequiredPlugin', name='SoftRobots.Inverse') # Where is SofaValidation ? => Deprecated Error in terminal
    pluginNode.addObject('RequiredPlugin', name='SoftRobots')
    pluginNode.addObject('RequiredPlugin', name='BeamAdapter')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.IO.Mesh')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.Engine.Generate')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.Mass')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.LinearSolver.Direct')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.Constraint.Lagrangian.Correction')  
    pluginNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') 
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Diffusion')
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop') # Needed to use components [FreeMotionAnimationLoop]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [SphereCollisionModel]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [GenericConstraintCorrection,UncoupledConstraintCorrection]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Generate') # Needed to use components [ExtrudeQuadsAndGenerateHexas]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') # Needed to use components [BoxROI]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshOBJLoader]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative') # Needed to use components [CGLinearSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [HexahedronFEMForceField]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic') # Needed to use components [HexahedronSetTopologyContainer,TriangleSetTopologyContainer]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Grid') # Needed to use components [RegularGridTopology]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]  


    rootNode.findData('gravity').value=[0, 0, 0];
    # rootNode.findData('gravity').value=[0, 0, 0];

    #visual dispaly
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')

    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight") # ne marche pas sans GUI
    
    rootNode.findData('dt').value= 0.01;
    
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')    

    rootNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')

    fichier =  'cylinder_16_parts.stl' 
    # fichier =  'parametric_cavity_sliced2.stl'

    bellowNode = createCavity(parent=rootNode,name_c="cavity",i=1,cavity_model=fichier,act_flag=1)

    points = rootNode.cavity2.meshLoader.position.value

    ConstrainCavity(points = points,parent=bellowNode,axis = 0,tolerance = 0.2)
    ConstrainCavity(points = points,parent=bellowNode,axis = 1,tolerance = 0.2)

    ## Elongation
    # ConstrainCavity(points = points,parent=bellowNode,axis = 2,tolerance = 0.2)

    bellowNode.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixMat3x3d")
    bellowNode.addObject('GenericConstraintCorrection')
    bellowNode.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)


    # return rootNode
