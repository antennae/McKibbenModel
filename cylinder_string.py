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


INVERSE = False


CONSTRAIN_BY_CABLE = True
CONSTRAIN_BY_SPRING = not CONSTRAIN_BY_CABLE


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

    triangleIndices = list(range(len(bellowNode.MeshLoader.triangles)))
    bellowNode.addObject(
        'SurfacePressureForceField',
        name='SPC',
        triangleIndices=triangleIndices,
        pressure=0,
    )

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

    # if inverse_flag == True:
    #     bellowNode.addObject(
    #         'SurfacePressureActuator',
    #         name='SPC',
    #         template='Vec3d',
    #         triangles='@chambreAMesh' + str(i + 1) + '.triangles',
    #         minPressure=0,
    #         maxPressure=300,
    #     )  # ,maxPressureVariation = 20)#,valueType=self.value_type)
    # elif inverse_flag == False:
    #     bellowNode.addObject(
    #         'SurfacePressureConstraint',
    #         name='SPC',
    #         triangles='@chambreAMesh' + str(i + 1) + '.triangles',
    #         value=0,
    #         minPressure=0,
    #         maxPressure=300,
    #         valueType="pressure",
    #     )  # ,maxPressureVariation = 20)#,

    # visu = bellowNode.addChild('Visu')
    # visu.addObject('OglModel', src=bellowNode.topology.getLinkPath())
    # visu.addObject('IdentityMapping')

    FixBasePosition(node=bellowNode)

    return bellowNode


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
        [i, (i + 1) % num_segments, num_segments] for i in range(num_segments)
    ]
    return points, triangles


def create_helix_mesh(
    radius,
    starting_z,
    num_points,
    num_turns,
    height,
    thread_num,
    phase_step,
    direction=1,
):
    angle_step = 2 * math.pi / num_points
    helix_points = []
    for i in range(num_points * num_turns):
        angle = i * angle_step
        z = starting_z + (i / num_points) * (height / num_turns)
        if direction == 1:
            x = radius * cos(angle + thread_num * phase_step)
            y = radius * sin(angle + thread_num * phase_step)
        if direction == -1:
            x = radius * sin(angle + thread_num * phase_step)
            y = radius * cos(angle + thread_num * phase_step)
        helix_points.append([x, y, z])
    return helix_points


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

    rootNode.addObject(
        'GenericConstraintSolver',
        maxIterations='100',
        tolerance='0.0000001',
    )
    rootNode.addObject(PressureController(pas=1, parent=pneumatic))

    points_node = pneumatic.getObject('MeshLoader')
    points = points_node.position.value

    tolerance = 0.05

    min_z = min(points, key=lambda p: p[2])[2]
    bottom_points = [
        i for i, p in enumerate(points) if abs(p[2] - min_z) <= tolerance
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
        for dir in [-1, 1]:
            # helix_points = []
            helix_points = create_helix_mesh(
                helix_radius,
                min_z,
                num_points,
                num_turns,
                height,
                n,
                phase_step,
                direction=dir,
            )

            edges = []
            for i in range(len(helix_points) - 1):
                edges.append([i, i + 1])

            helix_node = pneumatic.addChild(f'helix_{n}_{dir}')
            helix_node.addObject(
                'MeshTopology',
                name=f'helix_opology_{n}_{dir}',
                position=helix_points,
                edges=edges,
            )
            helix_node.addObject(
                'MechanicalObject',
                name=f'helix_points_{n}_{dir}',
                position=helix_points,
                showObject=True,
                showObjectScale=2,
            )

            if CONSTRAIN_BY_SPRING:
                helix_node.addObject(
                    'MeshSpringForceField',
                    name=f'helix_spring_{n}_{dir}',
                    stiffness=100000,
                    # damping=0.1,
                )
            if CONSTRAIN_BY_CABLE:
                helix_node.addObject(
                    'CableConstraint',
                    name=f'helix_cable_{n}_{dir}',
                    indices=list(range(len(helix_points))),
                    valueType='displacement',
                    value=0,
                    hasPullPoint=False,
                    drawPoints=False,
                    drawPullPoint=False,
                )
            helix_node.addObject('SkinningMapping')

    for radius in [1.6, 3]:
        for base_height in [height+min_z, min_z]:
            num_segments = 16

            # Top circular mesh
            top_points, top_triangles = create_circular_mesh(
                radius, num_segments, base_height
            )
            top_node = pneumatic.addChild(f'topCircularMesh_{radius}_at_{base_height}')
            top_node.addObject(
                'MeshTopology',
                name=f'Topology_{radius}_at_{base_height}',
                position=top_points,
                triangles=top_triangles,
            )
            top_node.addObject(
                'MechanicalObject',
                name=f'Points_{radius}_at_{base_height}',
                position=top_points,
                showObject=True,
                showObjectScale=2,
            )
            top_node.addObject(
                'MeshSpringForceField',
                name=f'Springs_{radius}_at_{base_height}',
                stiffness=10000000,
                # damping=0.1,
            )
            top_node.addObject('SkinningMapping')

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
