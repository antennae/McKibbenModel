# coding=utf-8

import Sofa
import Sofa.constants.Key as Key
import array
import numpy as np
from splib3.topology import remeshing as rf
from stl import mesh
from math import sin, cos, sqrt, acos, radians, dist, ceil
import math
import time

import ConstrainCylinder_Functions as constrain

CONSTRAIN = True
ELONGATION = (
    True  # True = modèle McKibben en élongation, sinon False, en compression
)
INVERSE = False

HELIX = True


def EffectorGoal(
    node, position, name, taille, solver=True
):  # => Factoriser avec EffectorGoal_Orientation ?
    goal = node.addChild(name)
    goal.addObject('MechanicalObject', name=name + 'M0', position=position)
    goal.addObject('SphereCollisionModel', radius=taille)
    if solver:
        goal.addObject('EulerImplicitSolver', firstOrder=True)
        goal.addObject(
            'CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10
        )  # ne sert à rien ?
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

    def __init__(self, goal_pas, node, name, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.stiffNode = node  # for the generic one
        self.position = self.stiffNode.getObject(name)

        self.pas = goal_pas

    def onKeypressedEvent(self, e):

        d = (self.position.position.value).copy()

        if e['key'] == Key.D:
            d[0][0] += self.pas
        if e['key'] == Key.C:
            d[0][0] -= self.pas

        if e['key'] == Key.F:
            d[0][1] += self.pas
        if e['key'] == Key.V:
            d[0][1] -= self.pas

        if e['key'] == Key.G:
            d[0][2] += self.pas
        if e['key'] == Key.B:
            d[0][2] -= self.pas

        self.position.position = [d[0]]


class PressureController(
    Sofa.Core.Controller
):  # TODO : ATTENTION : avec le dyn_flag, la pression max, min, et le pas sont multipié par dt. On redivise par dt pour les pressions soient bonnes à l'affichage. Tout est juste, mais du pint de vue du composant, tout est divisé en 2 (la moitié dans stiff_module, l'autre dans le composant = Pas cool, il faudrait mieux factoriser pour rendre le composant réutilisable)
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

    def __init__(self, pas, parent, node2='null', *args, **kwargs):

        Sofa.Core.Controller.__init__(self, args, kwargs)

        self.pressure = parent.getObject('SPC')
        self.flag = 0
        self.pas = pas
        self.max_pression = 30000

    def onKeypressedEvent(self, e):

        # SurfacePressureForceField
        pressureValue = self.pressure.pressure.value
        if e['key'] == Key.A:
            pressureValue += self.pas
            # print('===========D')
            if pressureValue > self.max_pression:
                pressureValue = self.max_pression
        if e['key'] == Key.Q:
            pressureValue -= self.pas
            if pressureValue < 0:
                pressureValue = 0

        self.pressure.pressure.value = pressureValue
        print('Pression cavité ', pressureValue)

        # # SurfacePressureActuator/Constraint
        # pressureValue = self.pressure.value.value[0]

        # if e['key'] == Key.A:
        #     pressureValue += self.pas
        #     # print('===========D')
        #     if pressureValue > self.max_pression:
        #         pressureValue = self.max_pression
        # if e['key'] == Key.Q:
        #     pressureValue -= self.pas
        #     if pressureValue < 0:
        #         pressureValue = 0

        # self.pressure.value = [pressureValue]
        # print('Pression cavité ', pressureValue)


def FixBasePosition(node):
    node.init()
    BaseBox = node.addObject(
        'BoxROI',
        name='boxROI_base',
        box=[-8, -8, -1, 8, 8, 1],
        drawBoxes=True,
        strict=False,
        drawTetrahedra=False,
    )  # si autom complète, mettre 8 dépendant des dimensions du robot
    BaseBox.init()
    print('selected : ')
    print(BaseBox.indices.value)
    node.addObject(
        'RestShapeSpringsForceField',
        points=BaseBox.indices.value,
        angularStiffness=1e5,
        stiffness=1e5,
    )  # pour accrocher la base du robot dans l'espace


def createCavity(
    parent, name_c, i, cavity_model, inverse_flag=False
):  # for v1 -------
    """
    name__c : name of the created node
    i : cavity number
    cavity_model : cavity model filename (should be .stl)
    inverse_flag = 0 : Inverse Control
    inverse_flag = 1 : Direct Control

    """
    bellowNode = parent.addChild(name_c + str(i + 1))

    bellowNode.addObject(
        'MeshSTLLoader',
        name='MeshLoader',
        filename=cavity_model,
    )
    bellowNode.addObject('MeshTopology', name='topology', src='@MeshLoader')
    bellowNode.addObject(
        'Vertex2Frame',
        position='@MeshLoader.position',
        normals='@MeshLoader.normals',
        name='engine_1',
        invertNormals='1',
        template='Rigid3d',
    )
    bellowNode.addObject(
        'MechanicalObject',
        name='chambreA' + str(i + 1),
        position='@engine_1.frames',
        template='Rigid3',
    )

    bellowNode.addObject('UniformMass', totalMass=1000, rayleighMass=0)

    # bellowNode.addObject(
    #     'TriangularBendingFEMForceField',
    #     thickness=1,  # 'm'
    #     youngModulus=1000,  # 'Pa'
    #     poissonRatio=0.49,
    # )

    bellowNode.addObject(
        'TriangularShellForceField',
        measure='Von Mises stress',
        showTriangle=True,
        thickness=1,  # 'm'
        showMeasuredValue=True,
        youngModulus=100,  # 'Pa'
        poissonRatio=0.49,
    )

    triangleIndices = list(range(len(bellowNode.MeshLoader.triangles)))
    bellowNode.addObject(
        'SurfacePressureForceField',
        name='SPC',
        triangleIndices=triangleIndices,
        pressure=0,
    )

    # visu = bellowNode.addChild('Visu')
    # visu.addObject('OglModel', src=bellowNode.topology.getLinkPath())
    # visu.addObject('IdentityMapping')


    # Can't use this because SurfacePressureModel in SoftRobot plugin
    # only support Vec3 type
    # chamberNode = bellowNode.addChild('ChamberNode')
    # chamberNode.addObject(
    #     'MeshTopology', name='topology', src='@../MeshLoader'
    # )
    # chamberNode.addObject(
    #     'MechanicalObject',
    #     name='chamber',
    #     template='Vec3d',
    # )

    # if inverse_flag == True:
    #     chamberNode.addObject(
    #         'SurfacePressureActuator',
    #         name='SPC',
    #         template='Vec3d',
    #         triangles='@MeshLoader.triangles',
    #         minPressure=0,
    #         maxPressure=3000,
    #     )  # ,maxPressureVariation = 20)#,valueType=self.value_type)
    # elif inverse_flag == False:
    #     chamberNode.addObject(
    #         'SurfacePressureConstraint',
    #         name='SPC',
    #         triangles='@MeshLoader.triangles',
    #         value=0,
    #         minPressure=0,
    #         maxPressure=3000,
    #         valueType='pressure',
    #     )  # ,maxPressureVariation = 20)#,
    # # chamberNode.addObject('RigidMapping')
    # chamberNode.addObject('BarycentricMapping')

    FixBasePosition(node=bellowNode)

    return bellowNode


def createScene(rootNode):
    # required plugins:
    pluginNode = rootNode.addChild('pluginNode')
    pluginNode.addObject(
        'RequiredPlugin',
        pluginName='SoftRobots.Inverse  \
                    SoftRobots  \
                    BeamAdapter  \
                    SOFA.Component.IO.Mesh  \
                    SOFA.Component.Engine.Generate\
                    SOFA.Component.Mass \
                    SOFA.Component.LinearSolver.Direct \
                    SOFA.Component.Constraint.Lagrangian.Correction \
                    Sofa.GL.Component.Rendering3D  \
                    Sofa.Component.Diffusion  \
                    Sofa.Component.AnimationLoop \
                    Sofa.Component.Collision.Geometry\
                    Sofa.Component.Constraint.Lagrangian.Solver\
                    Sofa.Component.Engine.Select \
                    Sofa.Component.LinearSolver.Iterative\
                    Sofa.Component.ODESolver.Backward  \
                    Sofa.Component.Setting  \
                    Sofa.Component.SolidMechanics.FEM.Elastic  \
                    Sofa.Component.SolidMechanics.Spring\
                    Sofa.Component.StateContainer \
                    Sofa.Component.Topology.Container.Dynamic \
                    Sofa.Component.Topology.Container.Grid  \
                    Sofa.Component.Visual \
                    Sofa.Component.Topology.Container.Constant\
                    Sofa.Component.Engine.Transform \
                    SofaShells  \
                    Sofa.Component.Mapping.Linear \
                    Sofa.Component.MechanicalLoad \
                    Sofa.Component.Constraint.Lagrangian.Correction\
                    Sofa.Component.IO.Mesh\
                    Sofa.Component.LinearSolver.Direct\
                    Sofa.Component.Mass',
    )

    rootNode.findData('gravity').value = [0, 0, 0]
    # rootNode.findData('gravity').value=[0, 0, 0];

    # visual dispaly
    rootNode.addObject(
        'VisualStyle',
        displayFlags='showVisualModels hideBehaviorModels showCollisionModels \
            hideBoundingCollisionModels showForceFields \
                showInteractionForceFields hideWireframe',
    )
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')

    rootNode.addObject(
        'OglSceneFrame', style='Arrows', alignment='TopRight'
    )  # ne marche pas sans GUI

    rootNode.findData('dt').value = 0.01

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    fichier = 'cylinder_16_parts.stl'
    # fichier =  'parametric_cavity_sliced2.stl'

    pneumatic = createCavity(
        parent=rootNode,
        name_c='cavity',
        i=1,
        cavity_model=fichier,
        inverse_flag=INVERSE,
    )

    if INVERSE:
        goal = EffectorGoal(
            node=rootNode,
            position=[0, 0, 42],
            name='goal',
            taille=2,
            solver=True,
        )
        controlledPoints = pneumatic.addChild('controlledPoints')
        controlledPoints.addObject(
            'MechanicalObject',
            name='actuatedPoints',
            template='Vec3',
            position=[0, 0, 42],
        )  # ,rotation=[0, 90 ,0]) # classic
        # controlledPoints.addObject('MechanicalObject', name='actuatedPoints', template='Rigid3',position=[0, 0, h_effector,0., 0., 0., 1.])#,rotation=[0, 90 ,0]) # rigid pour l'orientation
        controlledPoints.addObject(
            'PositionEffector',
            template='Vec3d',
            indices='0',
            effectorGoal='@../../goal/goalM0.position',
        )  # classic
        controlledPoints.addObject(
            'BarycentricMapping', mapForces=False, mapMasses=False
        )
        rootNode.addObject(
            'QPInverseProblemSolver',
            name='QP',
            printLog='0',
            saveMatrices=True,
            epsilon=0.01,
        )  # initialement epsilon = 0.001
        rootNode.addObject(
            GoalKeyboardController(goal_pas=1, node=goal, name='goalM0')
        )
    else:
        rootNode.addObject(
            'GenericConstraintSolver',
            maxIterations='100',
            tolerance='0.0000001',
        )
        rootNode.addObject(
            PressureController(pas=100, parent=pneumatic)
        )

    if CONSTRAIN:
        if HELIX:
            points_node = pneumatic.getObject('MeshLoader')
            points = points_node.position.value
            triangles = points_node.triangles.value

            tolerance = 0.05
            helix_element_list = []
            for triangle in triangles:
                p1, p2, p3 = (
                    points[triangle[0]],
                    points[triangle[1]],
                    points[triangle[2]],
                )
                if abs(p1[2] - p2[2]) > tolerance:
                    if dist([p1[0], p1[1]], [p2[0], p2[1]]) < tolerance:
                        pass
                    else:
                        helix_element_list.append([triangle[0], triangle[1]])
                if abs(p2[2] - p3[2]) > tolerance:
                    if dist([p2[0], p2[1]], [p3[0], p3[1]]) < tolerance:
                        pass
                    else:
                        helix_element_list.append([triangle[1], triangle[2]])
                if abs(p3[2] - p1[2]) > tolerance:
                    if dist([p1[0], p1[1]], [p3[0], p3[1]]) < tolerance:
                        pass
                    else:
                        helix_element_list.append([triangle[2], triangle[0]])

            stiffness = 100000
            i = 0

            for helix_element in helix_element_list:
                # print(helix_element[0])
                point1 = points[helix_element[0]]
                point2 = points[helix_element[1]]
                d = dist(point1, point2)
                helix_element_node = pneumatic.addChild(f'helix_element_{i}')
                helix_element_node.addObject(
                    'SpringForceField',
                    name='Springs',
                    stiffness=stiffness,
                    indices1=helix_element[0],
                    indices2=helix_element[1],
                    length=d,
                )  # damping='4'
                i += 1

            min_z = min(points, key=lambda p: p[2])[2]
            max_z = max(points, key=lambda p: p[2])[2]

            min_z_triangles = []
            max_z_triangles = []

            for triangle in triangles:
                p1, p2, p3 = (
                    points[triangle[0]],
                    points[triangle[1]],
                    points[triangle[2]],
                )
                if (
                    abs(p1[2] - min_z) < tolerance
                    and abs(p2[2] - min_z) < tolerance
                    and abs(p3[2] - min_z) < tolerance
                ):
                    min_z_triangles.append(triangle)
                if (
                    abs(p1[2] - max_z) < tolerance
                    and abs(p2[2] - max_z) < tolerance
                    and abs(p3[2] - max_z) < tolerance
                ):
                    max_z_triangles.append(triangle)

            def get_edges(triangles):
                edges = set()
                for triangle in triangles:
                    edges.add(tuple(sorted([triangle[0], triangle[1]])))
                    edges.add(tuple(sorted([triangle[1], triangle[2]])))
                    edges.add(tuple(sorted([triangle[2], triangle[0]])))
                return edges

            min_z_edges = get_edges(min_z_triangles)
            max_z_edges = get_edges(max_z_triangles)

            i = 0
            for edge in min_z_edges:
                point1 = points[edge[0]]
                point2 = points[edge[1]]
                d = dist(point1, point2)
                min_z_edge_node = pneumatic.addChild(f'min_z_edge_{edge}_{i}')
                min_z_edge_node.addObject(
                    'SpringForceField',
                    name='Springs',
                    stiffness=stiffness,
                    indices1=edge[0],
                    indices2=edge[1],
                    length=d,
                )
                i += 1

            i = 0
            for edge in max_z_edges:
                point1 = points[edge[0]]
                point2 = points[edge[1]]
                d = dist(point1, point2)
                max_z_edge_node = pneumatic.addChild(f'max_z_edge_{edge}_{i}')
                max_z_edge_node.addObject(
                    'SpringForceField',
                    name='Springs',
                    stiffness=stiffness,
                    indices1=edge[0],
                    indices2=edge[1],
                    length=d,
                )
                i += 1

        else:
            if ELONGATION == False:
                constrain.ConstrainFromCavity(
                    cavity_node=pneumatic, axis=0, tolerance=0.2
                )
                constrain.ConstrainFromCavity(
                    cavity_node=pneumatic, axis=1, tolerance=0.2
                )
            else:
                constrain.ConstrainFromCavity(
                    cavity_node=pneumatic, axis=2, tolerance=0.2
                )  # Elongation

    pneumatic.addObject(
        'SparseLDLSolver',
        name='ldlsolveur',
        template='CompressedRowSparseMatrixMat3x3d',
    )
    pneumatic.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)
    pneumatic.addObject('GenericConstraintCorrection')

    # return rootNode
