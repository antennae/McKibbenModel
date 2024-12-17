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
from splib3.numerics import Vec3, Quat, sdiv


import ConstrainCylinder_Functions as constrain

CONSTRAIN = True
ELONGATION = (
    True  # True = modèle McKibben en élongation, sinon False, en compression
)
INVERSE = False

HELIX = True

CONSTRAIN_BY_CABLE=True

if CONSTRAIN_BY_CABLE:
    CONSTRAIN_BY_SPRING=False
else:
    CONSTRAIN_BY_SPRING=True


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

    def Rigidify(targetObject, sourceObje
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

        # pressureValue = self.pressure.value.value[0]

        # if e["key"] == Key.A:
        #     pressureValue += self.pas
        #     # print('===========D')
        #     if pressureValue > self.max_pression:
        #         pressureValue = self.max_pression
        # if e["key"] == Key.Q:
        #     pressureValue -= self.pas
        #     if pressureValue < 0:
        #         pressureValue = 0

        # self.pressure.value = [pressureValue]
        # print('Pression cavité ', pressureValue)


############################################################################################################
# from stlib3.physics.mixedmaterial import rigidification
# Copied here to change template


# def getBarycenter(selectedPoints):
#     poscenter = [0.0, 0.0, 0.0]
#     if len(selectedPoints) != 0:
#         poscenter = sdiv(sum(selectedPoints), float(len(selectedPoints)))
#     return poscenter


# def Rigidify(targetObject, sourceObject, groupIndices, frames=None, name=None):
#     """Transform a deformable object into a mixed one containing both rigid and deformable parts.

#     :param targetObject: parent node where to attach the final object.
#     :param sourceObject: node containing the deformable object. The object should be following
#                          the ElasticMaterialObject template.
#     :param list groupIndices: array of array indices to rigidify. The length of the array should be equal to the number
#                               of rigid component.
#     :param list frames: array of frames. The length of the array should be equal to the number
#                         of rigid component. The orientation are given in eulerAngles (in degree) by passing
#                         three values or using a quaternion by passing four values.
#                         [[rx,ry,rz], [qx,qy,qz,w]]
#                         User can also specify the position of the frame by passing six values (position and orientation in degree)
#                         or seven values (position and quaternion).
#                         [[x,y,z,rx,ry,rz], [x,y,z,qx,qy,qz,w]]
#                         If the position is not specified, the position of the rigids will be the barycenter of the region to rigidify.
#     :param str name: specify the name of the Rigidified object, is none provided use the name of the SOurceObject.
#     """
#     if frames is None:
#         frames = [[0.0, 0.0, 0.0]] * len(groupIndices)

#     assert len(groupIndices) == len(frames), "size mismatch."

#     if name is None:
#         name = sourceObject.name

#     # sourceObject.reinit()
#     ero = targetObject.addChild(name)

#     allPositions = sourceObject.dofs.position.value
#     allIndices = list(range(len(allPositions)))

#     rigids = []
#     indicesMap = []

#     def mfilter(si, ai, pts):
#         tmp = []
#         for i in ai:
#             if i in si:
#                 tmp.append(pts[i])
#         return tmp

#     # get all the points from the source.
#     selectedIndices = []
#     for i in range(len(groupIndices)):
#         selectedPoints = mfilter(groupIndices[i], allIndices, allPositions)
#         if len(frames[i]) == 3:
#             orientation = Quat.createFromEuler(frames[i], inDegree=True)
#             poscenter = getBarycenter(selectedPoints)
#         elif len(frames[i]) == 4:
#             orientation = frames[i]
#             poscenter = getBarycenter(selectedPoints)
#         elif len(frames[i]) == 6:
#             orientation = Quat.createFromEuler(
#                 [frames[i][3], frames[i][4], frames[i][5]], inDegree=True
#             )
#             poscenter = [frames[i][0], frames[i][1], frames[i][2]]
#         elif len(frames[i]) == 7:
#             orientation = [
#                 frames[i][3],
#                 frames[i][4],
#                 frames[i][5],
#                 frames[i][6],
#             ]
#             poscenter = [frames[i][0], frames[i][1], frames[i][2]]
#         else:
#             Sofa.msg_error("Do not understand the size of a frame.")

#         rigids.append(poscenter + list(orientation))

#         selectedIndices += map(lambda x: x, groupIndices[i])
#         indicesMap += [i] * len(groupIndices[i])

#     otherIndices = list(filter(lambda x: x not in selectedIndices, allIndices))
#     Kd = {v: None for k, v in enumerate(allIndices)}
#     Kd.update({v: [0, k] for k, v in enumerate(otherIndices)})
#     Kd.update({v: [1, k] for k, v in enumerate(selectedIndices)})
#     indexPairs = [v for kv in Kd.values() for v in kv]

#     freeParticules = ero.addChild("DeformableParts")
#     freeParticules.addObject(
#         "MechanicalObject",
#         template="Rigid3",
#         name="dofs",
#         position=[allPositions[i] for i in otherIndices],
#     )

#     rigidParts = ero.addChild("RigidParts")
#     rigidParts.addObject(
#         "MechanicalObject",
#         template="Rigid3",
#         name="dofs",
#         reserve=len(rigids),
#         position=rigids,
#     )

#     rigidifiedParticules = rigidParts.addChild("RigidifiedParticules")
#     rigidifiedParticules.addObject(
#         "MechanicalObject",
#         template="Rigid3",
#         name="dofs",
#         position=[allPositions[i] for i in selectedIndices],
#     )
#     rigidifiedParticules.addObject(
#         "RigidMapping",
#         name="mapping",
#         globalToLocalCoords=True,
#         rigidIndexPerPoint=indicesMap,
#     )

#     if "solver" in sourceObject.objects:
#         sourceObject.removeObject(sourceObject.solver)
#     if "integration" in sourceObject.objects:
#         sourceObject.removeObject(sourceObject.integration)
#     if "correction" in sourceObject.objects:
#         sourceObject.removeObject(sourceObject.correction)

#     sourceObject.addObject(
#         "SubsetMultiMapping",
#         name="mapping",
#         template="Rigid3,Rigid3",
#         input=[
#             freeParticules.dofs.getLinkPath(),
#             rigidifiedParticules.dofs.getLinkPath(),
#         ],
#         output=sourceObject.dofs.getLinkPath(),
#         indexPairs=indexPairs,
#     )

#     rigidifiedParticules.addChild(sourceObject)
#     freeParticules.addChild(sourceObject)
#     return ero


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
    # print('selected : ')
    # print(BaseBox.indices.value)
    node.addObject(
        'RestShapeSpringsForceField',
        points=BaseBox.indices.value,
        angularStiffness=1e5,
        stiffness=1e5,
    )  # pour accrocher la base du robot dans l'espace


def RigidifyTopAndBase(parent_node, node):
    node.init()
    rigidPart = node.addChild('rigidPart')
    BaseBox = rigidPart.addObject(
        'BoxROI',
        name='boxROI_base',
        box=[-8, -8, -1, 8, 8, 1],
        drawBoxes=True,
        strict=False,
    )

    BaseTop = rigidPart.addObject(
        'BoxROI',
        name='boxROI_top',
        box=[-8, -8, 41, 8, 8, 43],
        drawBoxes=True,
        strict=False,
    )
    BaseBox.init()
    BaseTop.init()
    # points_to_rigidify = BaseBox.indices.value + BaseTop.indices.value
    points_to_rigidify = np.concatenate(
        (BaseBox.indices.value, BaseTop.indices.value)
    )
    # points_to_rigidify = np.concatenate(
    #     (BaseBox.pointsInROI.value, BaseTop.pointsInROI.value), axis=0
    # )
    # rigidPart.addObject(
    #     'MechanicalObject',
    #     name='ridigObject',
    #     position=points_to_rigidify,
    # )
    # rigidPart.addObject('RigidMapping')

    Rigidify(
        targetObject=parent_node,
        sourceObject=node,
        groupIndices=[points_to_rigidify],
        name='cap',
    )


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
    bellowNode.addObject('MeshTopology', name='container', src='@MeshLoader')
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
        name='dofs',
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

    # triangleIndices = list(range(len(bellowNode.MeshLoader.triangles)))
    # bellowNode.addObject(
    #     'SurfacePressureForceField',
    #     name='SPC',
    #     triangleIndices=triangleIndices,
    #     pressure=0,
    # )

    # bellowNode.addObject('Meshcontainer', src='@MeshLoader', name='Cavity')
    # bellowNode.addObject(
    #     'MechanicalObject', name='chambreA' + str(i + 1), rotation=[0, 0, 0]
    # )  # ,translation = [0,0,h_module*i]) # 90 on y
    # bellowNode.addObject('TriangleCollisionModel', moving='0', simulated='1')
    # bellowNode.addObject(
    #     'TriangleFEMForceField',
    #     template='Vec3',
    #     name='FEM',
    #     method='large',
    #     poissonRatio=0.49,
    #     youngModulus=100,
    #     thickness=5,
    # )  # stable youngModulus = 500 / réel ? = 103
    # bellowNode.addObject('UniformMass', totalMass=1000, rayleighMass=0)

    chamber_node = bellowNode.addChild('Chamber')
    chamber_node.addObject(
        'MechanicalObject',
        name='chamber',
        position='@../dofs.position',
        template='Vec3',
    )
    chamber_node.addObject(
        'SurfacePressureConstraint',
        name='SPC',
        triangles='@chambreAMesh' + str(i + 1) + '.triangles',
        value=0,
        minPressure=0,
        maxPressure=300,
        valueType='pressure',
    )  # ,maxPressureVariation = 20)#,
    chamber_node.addObject('SkinningMapping')
    # visu = bellowNode.addChild('Visu')
    # visu.addObject('OglModel', src=bellowNode.topology.getLinkPath())
    # visu.addObject('IdentityMapping')

    FixBasePosition(node=bellowNode)

    # RigidifyTopAndBase(parent_node=parent, node=bellowNode)

    return bellowNode


def createScene(rootNode):
    # required plugins:
    pluginNode = rootNode.addChild('pluginNode')
    pluginNode.addObject(
        'RequiredPlugin',
        pluginName='SoftRobots.Inverse  \
                    SoftRobots  \
                    BeamAdapter  \
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
                    Sofa.Component.Constraint.Lagrangian.Correction \
                    Sofa.Component.LinearSolver.Direct \
                    Sofa.Component.Mass \
                    Sofa.Component.Engine.Transform \
                    SofaShells  \
                    Sofa.Component.Mapping.Linear \
                    Sofa.Component.MechanicalLoad \
                    Sofa.Component.IO.Mesh',
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
    fichier = 'cylinder.stl'

    pneumatic = createCavity(
        parent=rootNode,
        name_c='cavity',
        i=1,
        cavity_model=fichier,
        inverse_flag=INVERSE,
    )

    # if INVERSE:
    #     goal = EffectorGoal(
    #         node=rootNode,
    #         position=[0, 0, 42],
    #         name='goal',
    #         taille=2,
    #         solver=True,
    #     )
    #     controlledPoints = pneumatic.addChild('controlledPoints')
    #     controlledPoints.addObject(
    #         'MechanicalObject',
    #         name='actuatedPoints',
    #         template='Vec3',
    #         position=[0, 0, 42],
    #     )  # ,rotation=[0, 90 ,0]) # classic
    #     controlledPoints.addObject(
    #         'MechanicalObject',
    #         name='actuatedPoints',
    #         template='Rigid3',
    #         position=[0, 0, h_effector, 0.0, 0.0, 0.0, 1.0],
    #     )  # ,rotation=[0, 90 ,0]) # rigid pour l'orientation
    #     controlledPoints.addObject(
    #         'PositionEffector',
    #         template='Vec3d',
    #         indices='0',
    #         effectorGoal='@../../goal/goalM0.position',
    #     )  # classic
    #     controlledPoints.addObject(
    #         'BarycentricMapping', mapForces=False, mapMasses=False
    #     )
    #     rootNode.addObject(
    #         'QPInverseProblemSolver',
    #         name='QP',
    #         printLog='0',
    #         saveMatrices=True,
    #         epsilon=0.01,
    #     )  # initialement epsilon = 0.001
    #     rootNode.addObject(
    #         GoalKeyboardController(goal_pas=1, node=goal, name='goalM0')
    #     )
    # else:
    rootNode.addObject(
        'GenericConstraintSolver',
        maxIterations='100',
        tolerance='0.0000001',
    )
    rootNode.addObject(PressureController(pas=1, parent=pneumatic))

    if CONSTRAIN:
        if HELIX:
            points_node = pneumatic.getObject('MeshLoader')
            points = points_node.position.value
            triangles = points_node.triangles.value

            tolerance = 0.05

            min_z = min(points, key=lambda p: p[2])[2]
            bottom_points = [
                i
                for i, p in enumerate(points)
                if abs(p[2] - min_z) <= tolerance
            ]
            helix_points = []
            num_turns = 2
            height = max(points, key=lambda p: p[2])[2] - min_z
            # num_points = len(bottom_points)
            num_points = 50
            angle_step = 2 * math.pi / num_points
            helix_radius = 3
            num_thread = 4
            phase_step = 2 * np.pi / num_thread

            for n in range(num_thread):
                helix_points = []
                for i in range(num_points * num_turns):
                    angle = i * angle_step
                    z = min_z + (i / num_points) * (height / num_turns)
                    x = helix_radius * cos(angle + n * phase_step)
                    y = helix_radius * sin(angle + n * phase_step)
                    helix_points.append([x, y, z])

                edges = []
                for i in range(len(helix_points) - 1):
                    edges.append([i, i + 1])

                helix_node = pneumatic.addChild(f'helixA_{n}')
                helix_node.addObject(
                    'MeshTopology',
                    name=f'helixA_opology_{n}',
                    position=helix_points,
                    edges=edges,
                )
                helix_node.addObject(
                    'MechanicalObject',
                    name=f'helixA_points_{n}',
                    position=helix_points,
                    showObject=True,
                    showObjectScale=2,
                )

                if CONSTRAIN_BY_SPRING:
                    helix_node.addObject(
                        'MeshSpringForceField',
                        name=f'helixA_spring_{n}',
                        stiffness=100000,
                        # damping=0.1,
                    )
                if CONSTRAIN_BY_CABLE:
                    helix_node.addObject(
                        'CableConstraint',
                        indices=list(range(len(helix_points))),
                        valueType='displacement',
                        value=0,
                        hasPullPoint=False,
                        drawPoints=False,
                        drawPullPoint=False,
                    )
                helix_node.addObject('SkinningMapping')

            for n in range(num_thread):
                helix_points = []
                for i in range(num_points * num_turns):
                    angle = i * angle_step
                    z = min_z + (i / num_points) * (height / num_turns)
                    x = helix_radius * sin(angle + n * phase_step)
                    y = helix_radius * cos(angle + n * phase_step)
                    helix_points.append([x, y, z])

                edges = []
                for i in range(len(helix_points) - 1):
                    edges.append([i, i + 1])

                helix_node = pneumatic.addChild(f'helixB_{n}')
                helix_node.addObject(
                    'MeshTopology',
                    name=f'helixB_topology_{n}',
                    position=helix_points,
                    edges=edges,
                )
                helix_node.addObject(
                    'MechanicalObject',
                    name=f'helixB_points_{n}',
                    position=helix_points,
                    showObject=True,
                    showObjectScale=2,
                )
                if CONSTRAIN_BY_SPRING:
                    helix_node.addObject(
                        'MeshSpringForceField',
                        name=f'helixB_spring_{n}',
                        stiffness=100000,
                        # damping=0.1,
                    )
                if CONSTRAIN_BY_CABLE:
                    helix_node.addObject(
                        'CableConstraint',
                        indices=list(range(len(helix_points))),
                        valueType='displacement',
                        value=0,
                        hasPullPoint=False,
                        drawPoints=False,
                        drawPullPoint=False,
                    )
                helix_node.addObject('SkinningMapping')

                # Create circular mesh on the top and bottom of the cylinder
                def create_circular_mesh(radius, num_segments, z):
                    angle_step = 2 * np.pi / num_segments
                    points = [
                        [
                            radius * np.cos(i * angle_step),
                            radius * np.sin(i * angle_step),
                            z,
                        ]
                        for i in range(num_segments)
                    ]
                    points.append([0, 0, z])  # center point
                    triangles = [
                        [i, (i + 1) % num_segments, num_segments]
                        for i in range(num_segments)
                    ]
                    return points, triangles

                for radius in [1.6, 3]:
                    # radius = 3.4  # Adjust as needed
                    num_segments = 16  # Adjust as needed

                    # Top circular mesh
                    top_points, top_triangles = create_circular_mesh(
                        radius, num_segments, height + min_z
                    )
                    top_node = pneumatic.addChild(f'topCircularMesh_{radius}')
                    top_node.addObject(
                        'MeshTopology',
                        name=f'topTopology_{radius}',
                        position=top_points,
                        triangles=top_triangles,
                    )
                    top_node.addObject(
                        'MechanicalObject',
                        name=f'topPoints_{radius}',
                        position=top_points,
                        showObject=True,
                        showObjectScale=2,
                    )
                    top_node.addObject(
                        'MeshSpringForceField',
                        name=f'topSprings_{radius}',
                        stiffness=10000000,
                        # damping=0.1,
                    )
                    top_node.addObject('SkinningMapping')

                    # Bottom circular mesh
                    bottom_points, bottom_triangles = create_circular_mesh(
                        radius, num_segments, min_z
                    )
                    bottom_node = pneumatic.addChild(
                        f'bottomCircularMesh_{radius}'
                    )
                    bottom_node.addObject(
                        'MeshTopology',
                        name=f'bottomTopology_{radius}',
                        position=bottom_points,
                        triangles=bottom_triangles,
                    )
                    bottom_node.addObject(
                        'MechanicalObject',
                        name=f'bottomPoints_{radius}',
                        position=bottom_points,
                        showObject=True,
                        showObjectScale=2,
                    )
                    bottom_node.addObject(
                        'MeshSpringForceField',
                        name=f'bottomSprings_{radius}',
                        stiffness=10000000,
                        # damping=0.1,
                    )
                    bottom_node.addObject('SkinningMapping')

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
    # pneumatic.addObject(
    #     'CGLinearSolver',
    #     name='linearSolver',
    #     iterations=1000,
    #     threshold=1e-5,
    #     tolerance=1e-5,
    #     template="CompressedRowSparseMatrixMat3x3d",
    # )
    pneumatic.addObject(
        'EulerImplicitSolver',
        firstOrder=False,
        rayleighStiffness=0.1,
        rayleighMass=0.1,
        vdamping=0,
    )
    pneumatic.addObject('GenericConstraintCorrection')

    # return rootNode
