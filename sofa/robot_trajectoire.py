# -*- coding: utf-8 -*-

import Sofa
import numpy
from splib.constants import Key
from math import sin, cos, sqrt
import csv
import codecs

PI = 3.14159265359

from splib.animation import AnimationManager

#####################################
# Definiton des parametres du robot #
#####################################
longueurArticulation= 0.4
largeurArticulation=0.2
stepTime      = 1.0e-3
stepDisplacement     = 2.0e-3
convergenceWaiting   = 0
toleranceConvergence = 30
deplacementTotal= 0
deplacementFinal=1
maillage=6
moduleYoung=1.4e3
coeffPoisson=0.48
masseJambe=1.864e-6
actuator_csv="actuators.csv"
with open(actuator_csv,"rb") as actuator_file:
    actuator_list=list(csv.reader(codecs.EncodedFile(actuator_file, 'utf8', 'utf_8_sig'),delimiter=';'))
    #print(actuator_list)
    #print(actuator_list[0][0])
    #print(len(actuator_list))
nb_Pt_traj=len(actuator_list)
Pt_traj=0

deplacement_plateforme_1="deplacement_plateforme_1_SOFA.txt"
deplacement_plateforme_2="deplacement_plateforme_2_SOFA.txt"
deplacement_doigt_1="deplacement_doigt_1_SOFA.txt"
deplacement_doigt_2="deplacement_doigt_2_SOFA.txt"


from splib.animation import AnimationManager
from compiler.ast import flatten
import rigidification

class BeamController(Sofa.PythonScriptController):

    def createGraph(self, node):
        print('---------- Entering createGraph ----------')
        self.node = node
        print('---------- Exiting createGraph  ----------')
        
    def reset(self):
        print('---------- Entering reset ----------')
        self.time = 0.0
        self.MechPosList=self.node.MecaModel.MechanicalObject.position
        self.stabTime = 0.0
        
        Write_deplacement_plateforme_1 = open(deplacement_plateforme_1, "w")
        Write_deplacement_plateforme_1.write(("init")+ "," +str((self.node.MecaModel.MechanicalObject.position[0]))+'\n')
        Write_deplacement_plateforme_1.close()
        Write_deplacement_plateforme_2 = open(deplacement_plateforme_2, "w")
        Write_deplacement_plateforme_2.write(("init")+ "," +str((self.node.MecaModel.MechanicalObject.position[1]))+'\n')
        Write_deplacement_plateforme_2.close()
        Write_deplacement_doigt_1 = open(deplacement_doigt_1, "w")
        Write_deplacement_doigt_1.write(("init")+ "," +str((self.node.MecaModel.EffectorNode.MechanicalObject.position[0]))+'\n')
        Write_deplacement_doigt_1.close()
        Write_deplacement_doigt_2 = open(deplacement_doigt_2, "w")
        Write_deplacement_doigt_2.write(("init")+ "," +str((self.node.MecaModel.EffectorNode.MechanicalObject.position[1]))+'\n')
        Write_deplacement_doigt_2.close()
        
        self.actuator_list=actuator_list
        self.convergenceWaiting=convergenceWaiting
        self.Pt_traj=Pt_traj
        self.nb_Pt_traj=nb_Pt_traj
        print('---------- Exiting reset  ----------')
        return 0

    def onBeginAnimationStep(self, dt):
        #print('---------- Entering onBeginAnimationStep ----------')
        self.time += dt
        if self.time> self.stabTime: #l'objectif est d'attendre la stabilite de la simu (etape non obligatoire)
            if self.convergenceWaiting>toleranceConvergence:
            
                if self.Pt_traj<self.nb_Pt_traj:
                    MechPosList=self.node.MecaModel.MechanicalObject.position
      
                    if self.Pt_traj==0:
                        print("entering loop premier point")
                        self.q1_init= self.MechPosList[12][1]
                        self.q2_init= self.MechPosList[7][0]
                        self.q3_init= self.MechPosList[13][1]
                        self.q4_init= self.MechPosList[6][0]
                        
                        print(str(self.q1_init))
                        
                    if self.Pt_traj>1:
                        self.node.MecaModel.FF.force=[0.0,0.0,0.0, 0.0,0.0,0.0] # enleve la force qui permet de sortir du plan au debut de la simu
                    
                    #recuperation des datas apres stabilisation
                    Write_deplacement_plateforme_1 = open(deplacement_plateforme_1, "a")
                    Write_deplacement_plateforme_1.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.MechanicalObject.position[0]))+'\n')
                    Write_deplacement_plateforme_1.close()
                    
                    Write_deplacement_plateforme_2 = open(deplacement_plateforme_2, "a")
                    Write_deplacement_plateforme_2.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.MechanicalObject.position[1]))+'\n')
                    Write_deplacement_plateforme_2.close()
                    
                    Write_deplacement_doigt_1 = open(deplacement_doigt_1, "a")
                    Write_deplacement_doigt_1.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.EffectorNode.MechanicalObject.position[0]))+'\n')
                    Write_deplacement_doigt_1.close()
                    
                    Write_deplacement_doigt_2 = open(deplacement_doigt_2, "a")
                    Write_deplacement_doigt_2.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.EffectorNode.MechanicalObject.position[1]))+'\n')
                    Write_deplacement_doigt_2.close()
                    
                    #print(self.actuator_list[self.Pt_traj])
                    
                    # nouvelles positions des actionneurs
                    MechPosList[6][0] = self.q4_init + float(self.actuator_list[self.Pt_traj][3]) #changement de la valeur de q4
                    MechPosList[7][0] = self.q2_init + float(self.actuator_list[self.Pt_traj][1]) #changement de la valeur de q2
                    MechPosList[12][1] = self.q1_init + float(self.actuator_list[self.Pt_traj][0]) #changement de la valeur de q1
                    MechPosList[13][1] = self.q3_init + float(self.actuator_list[self.Pt_traj][2]) #changement de la valeur de q3

                    self.node.MecaModel.MechanicalObject.position=MechPosList
                    #print(self.node.MecaModel.MechanicalObject.position[0]) # recupération de la position de la plateforme
                    #print(self.node.MecaModel.EffectorNode.MechanicalObject.position[0])

                    
                    
                    #reinitialisation de la convergence et incrementation des parametres
                    self.convergenceWaiting=0
                    self.Pt_traj+=1
                    print(str(self.Pt_traj))
                    
                elif self.Pt_traj==self.nb_Pt_traj:
                    print('entering last loop')
                    Write_deplacement_plateforme_1 = open(deplacement_plateforme_1, "a")
                    Write_deplacement_plateforme_1.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.MechanicalObject.position[0]))+'\n')
                    Write_deplacement_plateforme_1.close()
                    
                    Write_deplacement_plateforme_2 = open(deplacement_plateforme_2, "a")
                    Write_deplacement_plateforme_2.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.MechanicalObject.position[1]))+'\n')
                    Write_deplacement_plateforme_2.close()
                    
                    Write_deplacement_doigt_1 = open(deplacement_doigt_1, "a")
                    Write_deplacement_doigt_1.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.EffectorNode.MechanicalObject.position[0]))+'\n')
                    Write_deplacement_doigt_1.close()
                    
                    Write_deplacement_doigt_2 = open(deplacement_doigt_2, "a")
                    Write_deplacement_doigt_2.write(str((self.Pt_traj))+ "," +str((self.node.MecaModel.EffectorNode.MechanicalObject.position[1]))+'\n')
                    Write_deplacement_doigt_2.close()
                    self.Pt_traj+=1
                else :
                    print('---------Simulation ended---------')
                    
            self.convergenceWaiting+=1
        # print('---------- Exiting onBeginAnimationStep  ----------')
        return 0


def createScene(rootNode):
    
    rootNode.gravity='0 0 0'
    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('RequiredPlugin', pluginName='BeamAdapter')


    AnimationManager(rootNode)  
    rootNode.createObject('PythonScriptController', classname="BeamController")

    rootNode.createObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideMappings showForceFields')
    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('QPInverseProblemSolver')
    
    ########################################
    # Creation du modele mecanique du robot#
    ########################################
    
    MecaModelNode = rootNode.createChild('MecaModel')
    MecaModelNode.createObject('EulerImplicit', rayleighStiffness=0.02, 
                                    printLog=False, rayleighMass=0.01)
    MecaModelNode.createObject('SparseLDLSolver', name='ldl') 
    MecaModelNode.createObject('GenericConstraintCorrection', name='GCC', solverName='ldl')
    MecaModelNode.createObject('Mesh', edges=['0 1',  #definition des liens existant entre les elements rigides
                                              '0 1',
                                              '2 0',
                                              '3 0',
                                              '6 2',
                                              '6 3',
                                              '1 4',    
                                              '1 5',
                                              '4 7',
                                              '5 7',
                                              '0 8',
                                              '1 10',
                                              '0 9',
                                              '1 11',
                                              '8 12',
                                              '10 12',
                                              '9 13',
                                              '11 13'])
    MecaModelNode.createObject('MechanicalObject', template='Rigid3d', showObject='true', position=['-1.45  0.0 0.0  0 0 0 1',  # localisation des reperes de chacun des elements rigides
                                                                                                    ' 1.45  0.0 0.0 0.0 0 0 1',
                                                                                                    '-5.6  0.7 0  0 0 0 1',
                                                                                                    '-5.6 -0.7 0  0 0 0 1',
                                                                                                    ' 5.6 0.7 0  0 0 0 1',
                                                                                                    ' 5.6 -0.7 0  0 0 0 1',
                                                                                                    '-9.75  0.0 0  0 0 0 1',
                                                                                                    ' 9.75  0.0 0  0 0 0 1',
                                                                                                    '-1.45  4.15 0 0 0 0 1',
                                                                                                    '-1.45 -4.15 0 0 0 0 1',
                                                                                                    ' 1.45  4.15 0 0 0 0 1',
                                                                                                    ' 1.45 -4.15 0 0 0 0 1',
                                                                                                    ' 0     8.3  0 0 0 0 1',
                                                                                                    ' 0    -8.3  0 0 0 0 1'])
    
    
    MecaModelNode.createObject('BeamInterpolation', name='interpolation', crossSectionShape='rectangular', lengthZ='0.4', lengthY='0.4', #creation des beams entre les elements rigide en silicium
    dofsAndBeamsAligned='false', defaultYoungModulus='1.4e5' , DOF0TransformNode0=['1.25 0.7 -0.0  0 0 0 1 ',
                                                                                  '1.25 -0.7 -0.0  0 0 0 1 ',
                                                                                  '2.5 0.0 0  0 0 0 1',
                                                                                  '2.5 0 0  0 0 0 1',
                                                                                  '1.25 0.7 0  0 0 0 1', 
                                                                                  '1.25 -0.7 0 0 0 0 1',
                                                                                  '1.25 0.7 0  0 0 0 1',
                                                                                  '1.25 -0.7 0 0 0 0 1',
                                                                                  '2.5 0 0  0 0 0 1',
                                                                                  '2.5 0 0  0 0 0 1',
                                                                                  '0 1.25 -0.0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',  # orientation des beams avec les quaternions
                                                                                  '0 1.25 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -1.25 -0.0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -1.25 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' '],

                                                               DOF1TransformNode1=['-1.25 0.7 0  0 0 0 1',
                                                                                    '-1.25 -0.7 0  0 0 0 1' ,
                                                                                    '-1.25 0.7 -0.0  0 0 0 1 ',
                                                                                    '-1.25 -0.7 -0.0  0 0 0 1',
                                                                                    '-2.5 0 0  0 0 0 1',
                                                                                    '-2.5 0 0  0 0 0 1',
                                                                                    '-2.5 0 0  0 0 0 1 ',
                                                                                    '-2.5 0 0  0 0 0 1',
                                                                                    '-1.25 0.7 0 0 0 0 1',
                                                                                    '-1.25 -0.7 0 0 0 0 1',
                                                                                    '0 -2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '0 -2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '0 2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '0 2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '-1.45 -1.25 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '1.45 -1.25 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '-1.45 1.25 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '1.45 1.25 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' '])
    
    
    MecaModelNode.createObject('BeamInterpolation', name='interpolation2', crossSectionShape='rectangular', lengthZ='0.4', lengthY='0.6', #creation des beams entre les elements rigide en silicium
    dofsAndBeamsAligned='false', defaultYoungModulus='1.4e5' , DOF0TransformNode0=['1.25 0.7 -0.0  0 0 0 1 ',
                                                                                  '1.25 -0.7 -0.0  0 0 0 1 ',
                                                                                  '2.5 0.0 0  0 0 0 1',
                                                                                  '2.5 0 0  0 0 0 1',
                                                                                  '1.25 0.7 0  0 0 0 1', 
                                                                                  '1.25 -0.7 0 0 0 0 1',
                                                                                  '1.25 0.7 0  0 0 0 1',
                                                                                  '1.25 -0.7 0 0 0 0 1',
                                                                                  '2.5 0 0  0 0 0 1',
                                                                                  '2.5 0 0  0 0 0 1',
                                                                                  '0 1.25 -0.0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',  # orientation des beams avec les quaternions
                                                                                  '0 1.25 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -1.25 -0.0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -1.25 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                  '0 -2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' '],

                                                               DOF1TransformNode1=['-1.25 0.7 0  0 0 0 1',
                                                                                    '-1.25 -0.7 0  0 0 0 1' ,
                                                                                    '-1.25 0.7 -0.0  0 0 0 1 ',
                                                                                    '-1.25 -0.7 -0.0  0 0 0 1',
                                                                                    '-2.5 0 0  0 0 0 1',
                                                                                    '-2.5 0 0  0 0 0 1',
                                                                                    '-2.5 0 0  0 0 0 1 ',
                                                                                    '-2.5 0 0  0 0 0 1',
                                                                                    '-1.25 0.7 0 0 0 0 1',
                                                                                    '-1.25 -0.7 0 0 0 0 1',
                                                                                    '0 -2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '0 -2.5 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '0 2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '0 2.5 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '-1.45 -1.25 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '1.45 -1.25 0 0 0 '+str(sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '-1.45 1.25 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' ',
                                                                                    '1.45 1.25 0 0 0 '+str(-sin(PI/4.0))+' '+str(cos(PI/4.0))+' '])
    #creattion des masses des differents elements en gramme
    MecaModelNode.createObject('UniformMass',name='masse pince 0',indices='0',totalMass=['0.0043748'])
    MecaModelNode.createObject('UniformMass',name='masse pince 1',indices='1',totalMass=['0.0043748'])
    MecaModelNode.createObject('UniformMass',name='jambe 1',indices='2',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='jambe 2',indices='3',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='jambe 3',indices='4',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='jambe 4',indices='5',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='jambe 5',indices='8',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='jambe 6',indices='9',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='jambe 7',indices='10',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='jambe 8',indices='11',totalMass=['0.00283836'])
    MecaModelNode.createObject('UniformMass',name='masse base 1',indices='6',totalMass=['0.0099042'])
    MecaModelNode.createObject('UniformMass',name='masse base 2',indices='7',totalMass=['0.0099042'])
    MecaModelNode.createObject('UniformMass',name='masse base 3',indices='12',totalMass=['0.0099042'])
    MecaModelNode.createObject('UniformMass',name='masse base 4',indices='13',totalMass=['0.0099042'])
    
    MecaModelNode.createObject('FixedConstraint', indices='6 7 12 13') #fixe les deplacements des actionneurs suivant les directions voulues
    MecaModelNode.createObject('ConstantForceField', name='FF', indices='0 1',force=[0.0, 0.0, 1, 0.0, 0.0, 0.0], arrowSizeCoef=0.1)

    # MecaModelNode.createObject('PartialFixedConstraint', indices='12 13', fixedDirections='1 0 1 1 1 1')
    # MecaModelNode.createObject('PartialFixedConstraint', indices='0 1', fixedDirections='0 0 0 0 0 0 ')
    
    # deplacementActionneur='0.5'
    # MecaModelNode.createObject('SlidingActuator',name='q4', template='Rigid3d', indices='6', direction='1 0 0 0 0 0', maxForce='1000', minForce='-1000', maxDispVariation='0.002', maxPositiveDisp=deplacementActionneur, maxNegativeDisp=deplacementActionneur)
    # MecaModelNode.createObject('SlidingActuator', name='q2',template='Rigid3d', indices='7', direction='1 0 0 0 0 0', maxForce='1000', minForce='-1000', maxDispVariation='0.002', maxPositiveDisp=deplacementActionneur, maxNegativeDisp=deplacementActionneur)
    # MecaModelNode.createObject('SlidingActuator', name='q3',template='Rigid3d', indices='12', direction='0 1 0 0 0 0', maxForce='1000', minForce='-1000', maxDispVariation='0.002', maxPositiveDisp=deplacementActionneur, maxNegativeDisp=deplacementActionneur)
    # MecaModelNode.createObject('SlidingActuator', name='q1',template='Rigid3d', indices='13', direction='0 1 0 0 0 0', maxForce='1000', minForce='-1000', maxDispVariation='0.002', maxPositiveDisp=deplacementActionneur, maxNegativeDisp=deplacementActionneur)
    

    # MecaModelNode.createObject('PositionEffector', template='Rigid3d', useDirections='1 1 1 0 1 0', indices='0', effectorGoal='-1.45 0 2.7  0 0 0 1', maxShiftToTarget='0.01')
    # #MecaModelNode.createObject('PositionEffector', template='Rigid3d', useDirections='1 1 1 1 1 1', indices='0', effectorGoal=' -1.45 0 0  0 0 0 1', maxShiftToTarget='0.01')
    EffectorNode = MecaModelNode.createChild('EffectorNode')
    EffectorNode.createObject('MechanicalObject', template='Vec3d', position='1.43 0 2.7   -1.43 0 2.7' )
    #EffectorNode.createObject('MechanicalObject', template='Vec3d', position='0 0 0   0 0 0' )
    EffectorNode.createObject('RigidMapping',  rigidIndexPerPoint='0 1')
    EffectorNode.createObject('Sphere', radius='0.01')
    # EffectorNode.createObject('PositionEffector', template='Vec3d', useDirections='1 1 1', indices='0 1', effectorGoal='@../../GoalNode/mo_goal.position')
    
    MecaModelNode.createObject('AdaptiveBeamForceFieldAndMass', massDensity=0.97e-6, name='LinkForceField', interpolation='@interpolation')   
    
    ########################################
    # Creation des rendus visuels du robot #
    ########################################
    
    # VisuRigidNode0 = MecaModelNode.createChild('VisuRigid')
    # VisuRigidNode0.createObject('MeshSTLLoader', filename='plateforme.stl', name='loader',translation='1.45 0 -0.2',rotation='0 0 180', scale3d='1 1 1')
    # VisuRigidNode0.createObject('OglModel', src='@loader', name='visu')
    # VisuRigidNode0.createObject('RigidMapping', output='@visu', index='0')
    
    # VisuRigidNode1 = MecaModelNode.createChild('VisuRigid')
    # VisuRigidNode1.createObject('MeshSTLLoader', filename='plateforme.stl', name='loader',translation='-1.45 0 -0.2', scale3d='1 1 1')
    # VisuRigidNode1.createObject('OglModel', src='@loader', name='visu')
    # VisuRigidNode1.createObject('RigidMapping', output='@visu', index='1')
    
    VisuRigidNode0 = MecaModelNode.createChild('VisuPlateformeGauche')
    VisuRigidNode0.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\plateformeDoigt.stl', name='loader',translation='1.45 1.25 -2.5',rotation='0 0 180', scale3d='1 1 1')
    VisuRigidNode0.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode0.createObject('RigidMapping', output='@VisuPlateformeGauche', index='0')
    
    VisuRigidNode1 = MecaModelNode.createChild('VisuPlateformeDroite')
    VisuRigidNode1.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\plateformeDoigt.stl', name='loader',translation='-1.45 -1.25 -2.5', scale3d='1 1 1')
    VisuRigidNode1.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode1.createObject('RigidMapping', output='@visu', index='1')
    
    VisuRigidNode2 = MecaModelNode.createChild('VisuJambe1')
    VisuRigidNode2.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-1.5 0.2 -0.2',rotation='90 0 0', scale3d='1 1 1')
    VisuRigidNode2.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode2.createObject('RigidMapping', output='@visu', index='2')
    
    VisuRigidNode3 = MecaModelNode.createChild('VisuJambe2')
    VisuRigidNode3.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-1.5 0.2 -0.2',rotation='90 0 0', scale3d='1 1 1')
    VisuRigidNode3.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode3.createObject('RigidMapping', output='@visu', index='3')
    
    VisuRigidNode4 = MecaModelNode.createChild('VisuJambe3')
    VisuRigidNode4.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-1.5 0.2 -0.2',rotation='90 0 0', scale3d='1 1 1')
    VisuRigidNode4.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode4.createObject('RigidMapping', output='@visu', index='4')
    
    VisuRigidNode5 = MecaModelNode.createChild('VisuJambe4')
    VisuRigidNode5.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-1.5 0.2 -0.2',rotation='90 0 0', scale3d='1 1 1')
    VisuRigidNode5.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode5.createObject('RigidMapping', output='@visu', index='5')
    
    VisuRigidNode6 = MecaModelNode.createChild('VisuBaseXNeg')
    VisuRigidNode6.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot/base.stl', name='loader',translation='1.25 -1.75 -0.2',rotation='0 0 90', scale3d='1 1 1')
    VisuRigidNode6.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode6.createObject('RigidMapping', output='@visu', index='6')
    
    VisuRigidNode7 = MecaModelNode.createChild('VisuBaseXPos')
    VisuRigidNode7.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot/base.stl', name='loader',translation='1.25 -1.75 -0.2',rotation='0 0 90', scale3d='1 1 1')
    VisuRigidNode7.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode7.createObject('RigidMapping', output='@visu', index='7')
    
    VisuRigidNode8 = MecaModelNode.createChild('VisuJambe5')
    VisuRigidNode8.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-0.2 -1.5 -0.2', rotation='90 0 90', scale3d='1 1 1')
    VisuRigidNode8.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode8.createObject('RigidMapping', output='@visu', index='8')
    
    VisuRigidNode9 = MecaModelNode.createChild('VisuJambe6')
    VisuRigidNode9.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-0.2 -1.5 -0.2', rotation='90 0 90', scale3d='1 1 1')
    VisuRigidNode9.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode9.createObject('RigidMapping', output='@visu', index='9')
    
    VisuRigidNode10 = MecaModelNode.createChild('VisuJambe7')
    VisuRigidNode10.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-0.2 -1.5 -0.2', rotation='90 0 90', scale3d='1 1 1')
    VisuRigidNode10.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode10.createObject('RigidMapping', output='@visu', index='10')
    
    VisuRigidNode11 = MecaModelNode.createChild('VisuJambe8')
    VisuRigidNode11.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot\jambe.stl', name='loader',translation='-0.2 -1.5 -0.2', rotation='90 0 90', scale3d='1 1 1')
    VisuRigidNode11.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode11.createObject('RigidMapping', output='@visu', index='11')
    
    VisuRigidNode12 = MecaModelNode.createChild('VisuBaseYNeg')
    VisuRigidNode12.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot/baseLarge2.stl', name='loader',translation='-2.25 -1.25 -0.2', scale3d='1 1 1')
    VisuRigidNode12.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode12.createObject('RigidMapping', output='@visu', index='12')
    
    VisuRigidNode13 = MecaModelNode.createChild('VisuBaseYPos')
    VisuRigidNode13.createObject('MeshSTLLoader', filename='G:\Utilisateurs\maxence.leveziel\Desktop\SOFA_v19.06.99_custom_Win64_v8.1\MiGriBot/baseLarge2.stl', name='loader',translation='-2.25 -1.25 -0.2', scale3d='1 1 1')
    VisuRigidNode13.createObject('OglModel', src='@loader', name='visu')
    VisuRigidNode13.createObject('RigidMapping', output='@visu', index='13')
   
    return rootNode