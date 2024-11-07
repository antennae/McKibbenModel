"""
Auteur : Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2024
Propriétaire : Université de Lille - CNRS 
License : Non définie, mais développé dans une démarche Open-Source et Logiciel Libre avec volonté de partage et de travail collaboratif. Développé dans un but non-marchand, en cas d'utilisation commerciale, merci de minimiser les prix et de favoriser le partage gratuit de tout ce qui peut l'être. A utiliser dans des buts prenant en compte les questions éthiques et morales (si possible non-militaire, ne rentrant pas dans le cadre de compétition, de monopole, ou de favorisation d'interets privés).
"""

 # coding=utf-8

import Sofa
import Sofa.constants.Key as Key
import array
import numpy as np
from splib3.topology import remeshing as rf
from stl import mesh
from math import sin,cos, sqrt, acos, radians, dist, ceil
import math
import time

import ConstrainCylinder_Functions as constrain

PLUGIN = True # True will required the updated version of soft robot plugin (python prefab)
CONSTRAIN = True
ELONGATION = True # True = modèle McKibben en élongation, sinon False, en compression
INVERSE = True

if PLUGIN == True :
    from softrobots.actuators import pneumatic as pb
    from softrobots.actuators import constrain as cons
    from softrobots.inverse.actuators import pneumatic as i_pb
    from softrobots.inverse.effectors import effectorGoal as eG

def EffectorGoal(node, position,name,taille,solver=True): # => Factoriser avec EffectorGoal_Orientation ?
    goal = node.addChild(name)
    goal.addObject('MechanicalObject', name=name + 'M0', position=position)
    goal.addObject('SphereCollisionModel', radius=taille)
    if solver :
        goal.addObject('EulerImplicitSolver', firstOrder=True)
        goal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10) # ne sert à rien ?
        goal.addObject('UncoupledConstraintCorrection')
    # goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
    return goal

class GoalKeyboardController(Sofa.Core.Controller):
    """
    FR : 
    Pour controller la position du goal point selectionné (passé en argument) avec le clavier
        INPUT : 
        goal_pas = pas de déplacement de l'effecteur, en mm
        node = noeud du goal point
        name = nom de l'objet du goal point ("goalM0")

    EN :
    To control the position of the selected goal point (passed as an argument) with the keyboard
         INPUT:
         goal_pas = end-effector displacement step, in mm
         node = goal point node
         name = name of the goal point object ("goalM0")

    Exemple : rootNode.addObject(GoalKeyboardController(goal_pas = goal_pas,node = goal2,name = 'goal2M0')) # for goal with shift

    """

    def __init__(self,goal_pas,node,name,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.stiffNode = node # for the generic one
        self.position = self.stiffNode.getObject(name)

        self.pas = goal_pas

    def onKeypressedEvent(self,e):

        d = (self.position.position.value).copy()

        if e["key"] == Key.D:
            d[0][0] += self.pas  
        if e["key"] == Key.C:
            d[0][0] -= self.pas  

        if e["key"] == Key.F:
            d[0][1] += self.pas  
        if e["key"] == Key.V:
            d[0][1] -= self.pas  

        if e["key"] == Key.G:
            d[0][2] += self.pas  
        if e["key"] == Key.B:
            d[0][2] -= self.pas 

        self.position.position = [d[0]]


class PressureController(Sofa.Core.Controller): # TODO : ATTENTION : avec le dyn_flag, la pression max, min, et le pas sont multipié par dt. On redivise par dt pour les pressions soient bonnes à l'affichage. Tout est juste, mais du pint de vue du composant, tout est divisé en 2 (la moitié dans stiff_module, l'autre dans le composant = Pas cool, il faudrait mieux factoriser pour rendre le composant réutilisable)
    """
        FR :
        Fonction pour pouvoir modifier les pressions appliqués par le clavier
            INPUT : 
            pas = step, incrément en pression (kPa) à chaque frappe de clavier
            module = variable stiff qui contient toutes les données du robot
            parent = noeud parent des cavités pour s'y connecter

        EN :
        Function to be able to modify the pressures applied by the keyboard
             INPUT:
             pas = step, increment in pressure (kPa) with each keystroke
             module = variable stiff which contains all the data of the robot
             parent = parent node of the cavities to connect them

        Exemple : rootNode.addObject(StiffController(pas=pas,module = stiff,parent = stiff_flop))
    """

    def __init__(self,pas,parent,node2 = "null",*args, **kwargs):

            Sofa.Core.Controller.__init__(self,args,kwargs)

            self.pressure = parent.getObject('SPC')
            self.flag = 0;
            self.pas = pas
            self.max_pression = 300
            

    def onKeypressedEvent(self,e):
    
            pressureValue = self.pressure.value.value[0]

            if e["key"] == Key.A:
                pressureValue += self.pas
                # print('===========D')
                if pressureValue > self.max_pression:
                    pressureValue= self.max_pression
            if e["key"] == Key.Q:
                pressureValue -= self.pas
                if pressureValue < 0:
                    pressureValue = 0
                        
            self.pressure.value =  [pressureValue]
            print('Pression cavité ', pressureValue)        

def FixBasePosition(node):
    node.init()
    BaseBox = node.addObject('BoxROI', name='boxROI_base', box=[-8, -8, -1, 8, 8, 1], drawBoxes=True, strict=False,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
    BaseBox.init()
    print("selected : ")
    print(BaseBox.indices.value)
    node.addObject('RestShapeSpringsForceField', points=BaseBox.indices.value, angularStiffness=1e5, stiffness=1e5) # pour accrocher la base du robot dans l'espace


def createCavity(parent,name_c,i,cavity_model,inverse_flag = False): # for v1 -------

    """
    name__c : name of the created node
    i : cavity number
    cavity_model : cavity model filename (should be .stl)
    inverse_flag = 0 : Inverse Control
    inverse_flag = 1 : Direct Control

    """
    bellowNode = parent.addChild(name_c+str(i+1))
    MeshLoad = bellowNode.addObject('MeshSTLLoader', filename=cavity_model, flipNormals='0', triangulate='true', name='MeshLoader',rotation=[0,0,0], translation=[0, 0,0])#, rotation=[self.ang_dec*self.i_cavity,0,0] if pre-rotated 3D model
    MeshLoad.init()
    points  = MeshLoad.position.value
    triangles = MeshLoad.triangles.value
    bellowNode.addObject('MeshTopology', src='@MeshLoader', name='Cavity')
    bellowNode.addObject('MechanicalObject', name='chambreA'+str(i+1),rotation=[0, 0 , 0])#,translation = [0,0,h_module*i]) # 90 on y
    bellowNode.addObject('TriangleCollisionModel', moving='0', simulated='1')
    bellowNode.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.49,  youngModulus=100, thickness = 5) # stable youngModulus = 500 / réel ? = 103
    bellowNode.addObject('UniformMass', totalMass=1000, rayleighMass = 0)

    if inverse_flag == True :
        bellowNode.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh'+str(i+1)+'.triangles',minPressure = 0,maxPressure = 300)#,maxPressureVariation = 20)#,valueType=self.value_type)
    elif  inverse_flag == False :
        bellowNode.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMesh'+str(i+1)+'.triangles', value=0,minPressure = 0,maxPressure = 300, valueType="pressure" )#,maxPressureVariation = 20)#,

    FixBasePosition(node = bellowNode)

    return bellowNode

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


    fichier =  'cylinder_16_parts.stl' 
    # fichier =  'parametric_cavity_sliced2.stl'

    if PLUGIN == False : ##### Version du code autonome, appelant les fonctions écrites ici #################################################### 1

        pneumatic = createCavity(parent=rootNode,name_c="cavity",i=1,cavity_model=fichier,inverse_flag=INVERSE)

        if INVERSE :
            goal = EffectorGoal(node=rootNode, position = [0,0,42],name = 'goal',taille = 2,solver = True)
            controlledPoints = pneumatic.addChild('controlledPoints')
            controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Vec3",position=[0, 0, 42])#,rotation=[0, 90 ,0]) # classic
            # controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Rigid3",position=[0, 0, h_effector,0., 0., 0., 1.])#,rotation=[0, 90 ,0]) # rigid pour l'orientation
            controlledPoints.addObject('PositionEffector', template="Vec3d", indices='0', effectorGoal="@../../goal/goalM0.position") # classic
            controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)
            rootNode.addObject('QPInverseProblemSolver', name="QP", printLog='0', saveMatrices = True ,epsilon = 0.01) # initialement epsilon = 0.001
            rootNode.addObject(GoalKeyboardController(goal_pas = 1,node = goal,name = 'goalM0'))
        else :
            rootNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')
            rootNode.addObject(PressureController(pas=10,parent = pneumatic))

        if CONSTRAIN :
            if ELONGATION == False :
                constrain.ConstrainFromCavity(cavity_node=pneumatic,axis = 0,tolerance = 0.2)
                constrain.ConstrainFromCavity(cavity_node=pneumatic,axis = 1,tolerance = 0.2)
            else :
                constrain.ConstrainFromCavity(cavity_node=pneumatic,axis = 2,tolerance = 0.2) ## Elongation

    else : ##### Version du code dépendant du code déporté dans les prefab python (a update TODO) ################################################# 1
        if INVERSE :
            pneumatic = i_pb.PneumaticCavity(surfaceMeshFileName=fichier, attachedTo=rootNode)
            pneumatic.addObject('TriangleCollisionModel', moving='0', simulated='1') # For visualisation
            pneumatic.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.49,  youngModulus=100, thickness = 5) # stable youngModulus = 500 / réel ? = 103
            pneumatic.addObject('UniformMass', totalMass=1000, rayleighMass = 0)
            FixBasePosition(node = pneumatic)
            goal = eG.CompleteEffectorGoal(attachedTo=rootNode, bodyNode = pneumatic, goal_position = [0,0,42], associated_position = [0,0,42], name = 'goal')
            rootNode.addObject('QPInverseProblemSolver', name="QP", printLog='0', saveMatrices = True ,epsilon = 0.01) # initialement epsilon = 0.001
            rootNode.addObject(GoalKeyboardController(goal_pas = 1,node = goal,name = 'goal'))
        else :
            pneumatic = pb.PneumaticCavity(surfaceMeshFileName=fichier, attachedTo=rootNode)
            pneumatic.addObject('TriangleCollisionModel', moving='0', simulated='1') # For visualisation
            pneumatic.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.49,  youngModulus=100, thickness = 5) # stable youngModulus = 500 / réel ? = 103
            pneumatic.addObject('UniformMass', totalMass=1000, rayleighMass = 0)
            FixBasePosition(node = pneumatic)
            rootNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')
            rootNode.addObject(pb.PressureController(pas=10,parent = pneumatic))

        # if CONSTRAIN :
        #     print("tessst")
        #     if ELONGATION == False :
        #         cons.ConstrainFromCavity(cavity_node=pneumatic,axis = 0,tolerance = 0.2)
        #         cons.ConstrainFromCavity(cavity_node=pneumatic,axis = 1,tolerance = 0.2)
        #     else :
        #         cons.ConstrainFromCavity(cavity_node=pneumatic,axis = 2,tolerance = 0.2) ## Elongation
           ######################################################################################################################################## 1




    pneumatic.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixMat3x3d")
    pneumatic.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)
    pneumatic.addObject('GenericConstraintCorrection')



    # return rootNode
