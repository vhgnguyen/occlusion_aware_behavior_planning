from PyQt5 import QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QGroupBox,
                             QVBoxLayout, QHBoxLayout, QPushButton,
                             QLineEdit, QGridLayout, QLabel)
import numpy as np
import matplotlib.pyplot as plt
import tikzplotlib
from matplotlib.patches import Ellipse, Polygon
import helper as helper


class InfoPanel(QGroupBox):

    def __init__(self, core, parent=None):
        super(InfoPanel, self).__init__(parent)
        self.core = core
        self.setTitle("Information panel")
        self.setMaximumSize(400, 900)
        self.setMinimumSize(400, 900)
        self.mainLayout = QVBoxLayout()
        self.addInfoBox()
        self.mainLayout.addStretch()
        self.setLayout(self.mainLayout)
        self._plotScene, self._ax = None, None

    def addInfoBox(self):
        # information control
        self.infoBox = QGroupBox()
        self.infoGrid = QGridLayout()

        self.infoGrid.addWidget(QLabel("Simulation time [s]"), 0, 0)
        self.simulationTimeValue = QLabel()
        self.simulationTimeValue.setMaximumWidth(100)
        self.infoGrid.addWidget(self.simulationTimeValue, 0, 1)

        self.infoGrid.addWidget(QLabel("Current simulation timestamp [s]"), 1, 0)
        self.currentTimeValue = QLabel()
        self.simulationTimeValue.setMaximumWidth(100)
        self.infoGrid.addWidget(self.currentTimeValue, 1, 1)

        # ego control
        self.infoGrid.addWidget(QLabel("Velocity"), 2, 0)
        self.egoVelocityValue = QLabel()
        self.egoVelocityValue.setMaximumWidth(100)
        self.infoGrid.addWidget(self.egoVelocityValue, 2, 1)

        self.infoGrid.addWidget(QLabel("Acceleration"), 3, 0)
        self.egoAccValue = QLabel()
        self.egoAccValue.setMaximumWidth(100)
        self.infoGrid.addWidget(self.egoAccValue, 3, 1)

        self.infoBox.setLayout(self.infoGrid)
        self.mainLayout.addWidget(self.infoBox)

        # plot button
        self.graphBox = QGroupBox()
        self.graphGrid = QGridLayout()

        self.pathName = QLineEdit()
        self.pathName.setText('result/')
        self.pathName.setMaximumWidth(400)
        self.graphGrid.addWidget(self.pathName, 0, 0)

        self.plotDButton = QPushButton("Show dynamic plot")
        self.plotDButton.clicked.connect(self.on_plotD_clicked)
        self.plotDButton.setMaximumWidth(300)
        self.graphGrid.addWidget(self.plotDButton, 1, 0)

        self.saveDynamicName = QLineEdit()
        self.saveDynamicName.setText("dynamic-time.txt")
        self.saveDynamicName.setMaximumWidth(400)
        self.graphGrid.addWidget(self.saveDynamicName, 2, 0)

        self.saveDynamicButton = QPushButton("Save")
        self.saveDynamicButton.clicked.connect(self.on_saveD_clicked)
        self.saveDynamicButton.setMaximumWidth(100)
        self.graphGrid.addWidget(self.saveDynamicButton, 2, 1)

        self.plotDDButton = QPushButton("Show dynamic-distance plot")
        self.plotDDButton.clicked.connect(self.on_plotDD_clicked)
        self.plotDDButton.setMaximumWidth(300)
        self.graphGrid.addWidget(self.plotDDButton, 3, 0)

        self.saveDynamicDistanceName = QLineEdit()
        self.saveDynamicDistanceName.setText("dynamic-distance.txt")
        self.saveDynamicDistanceName.setMaximumWidth(400)
        self.graphGrid.addWidget(self.saveDynamicDistanceName, 4, 0)

        self.saveDynamicDistanceButton = QPushButton("Save")
        self.saveDynamicDistanceButton.clicked.connect(self.on_saveDD_clicked)
        self.saveDynamicDistanceButton.setMaximumWidth(100)
        self.graphGrid.addWidget(self.saveDynamicDistanceButton, 4, 1)

        self.plotRButton = QPushButton("Show risk plot")
        self.plotRButton.clicked.connect(self.on_plotR_clicked)
        self.plotRButton.setMaximumWidth(300)
        self.graphGrid.addWidget(self.plotRButton, 5, 0)

        self.saveRiskName = QLineEdit()
        self.saveRiskName.setText("risk.txt")
        self.saveRiskName.setMaximumWidth(400)
        self.graphGrid.addWidget(self.saveRiskName, 6, 0)

        self.saveRiskButton = QPushButton("Save")
        self.saveRiskButton.clicked.connect(self.on_saveRisk_clicked)
        self.saveRiskButton.setMaximumWidth(100)
        self.graphGrid.addWidget(self.saveRiskButton, 6, 1)

        self.graphBox.setLayout(self.graphGrid)
        self.mainLayout.addWidget(self.graphBox)

        self.plotSceneButton = QPushButton("Show scene plot")
        self.plotSceneButton.clicked.connect(self.on_plotScene_clicked)
        self.plotSceneButton.setMaximumWidth(300)
        self.graphGrid.addWidget(self.plotSceneButton, 7, 0)

        self.saveSceneName = QLineEdit()
        self.saveSceneName.setText("scene.tex")
        self.saveSceneName.setMaximumWidth(400)
        self.graphGrid.addWidget(self.saveSceneName, 8, 0)

        self.saveSceneButton = QPushButton("Save")
        self.saveSceneButton.clicked.connect(self.on_saveScene_clicked)
        self.saveSceneButton.setMaximumWidth(100)
        self.graphGrid.addWidget(self.saveSceneButton, 8, 1)

        self.graphBox.setLayout(self.graphGrid)
        self.mainLayout.addWidget(self.graphBox)

    def update(self):
        if self.core._egoCar is not None:
            self.currentTimeValue.setText(str(self.core.getCurrentTime()) + " s")
            self.simulationTimeValue.setText(str(self.core.getSimulationTime()) + " s")
            self.egoVelocityValue.setText(str(round(self.core.getCurrentVelocity(), 2)) + " m/s")
            self.egoAccValue.setText(str(round(self.core.getCurrentAcceleration(), 2)) + " m/s<sup>2</sup>")

    def on_plotD_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotDynamic()

    def on_plotR_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotRisk()

    def on_plotDD_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotDynamicDistance()

    def on_plotScene_clicked(self):
        if self.core._egoCar is not None:
            self.plotScene()

    def on_saveD_clicked(self):
        if self.core._egoCar is not None:
            self.core.saveDynamic(
                self.pathName.text(),
                self.saveDynamicName.text())

    def on_saveDD_clicked(self):
        if self.core._egoCar is not None:
            self.core.saveDynamicDistance(
                self.pathName.text(),
                self.saveDynamicDistanceName.text())

    def on_saveRisk_clicked(self):
        if self.core._egoCar is not None:
            self.core.saveRisk(
                self.pathName.text(),
                self.saveRiskName.text())

    def on_saveScene_clicked(self):
        if self.core._egoCar is not None:
            tikzplotlib.save(
                self.pathName.text()+self.saveSceneName.text(),
                axis_height='10cm',
                axis_width='16cm')

    def plotScene(self):
        fig, ax = plt.subplots(figsize=(14, 6))
        pose = self.core.getCurrentEgoPos()

        self.plotRoad(ax=ax)
        self.plotFOV(ax=ax)
        self.plotStaticObject(ax=ax)
        self.plotPedestrianCross(ax=ax)
        self.plotEgoVehicle(ax=ax)
        self.plotPedestrian(ax=ax)
        self.plotVehicle(ax=ax)

        helper.handleLegend(ax=plt)
        ax.axis('equal')
        ax.set_xlim(pose[0]-40, pose[0]+40)
        ax.set_ylim(pose[1]-20, pose[1]+20)
        ax.set_xlabel("x[m]")
        ax.set_ylabel("y[m]")
        self._plotScene, self._ax = fig, ax
        plt.show()

    def plotPedestrian(self, ax):
        # plot other pedestrian
        pedesList = self.core.exportCurrentPedestrian()
        hypoPList = self.core.exportHypoPedestrian()

        for hypoP in hypoPList:
            hp = hypoP['p']
            a = 1
            b = len(hp)
            for i in range(0, b, int(b/5)):
                hpp = hp[i]
                a = max(a-0.15, 0.1)
                pose = hpp['pos']
                lw = hpp['std']
                helper.plotEllipse(
                    x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                    facecolor='pink', edgecolor='tab:pink', alpha=a, ax=ax)

            hc = hypoP['c']
            helper.plotPolygon(
                hc['poly'], facecolor='pink', edgecolor='k',
                alpha=1, label="hypothesis pedestrian", ax=ax,
                heading=True, hcolor='k')

        for pedes in pedesList:
            c = pedes['c']
            p = pedes['p']

            a = 1
            b = len(p)
            for i in range(0, b, int(b/5)):
                pp = p[i]
                a = max(a-0.15, 0.1)
                pose = pp['pos']
                lw = pp['std']
                if c['visible']:
                    helper.plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='lime', edgecolor='tab:pink', alpha=a, ax=ax)
                else:
                    helper.plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='salmon', edgecolor='tab:pink', alpha=a, ax=ax)

            if c['visible']:
                helper.plotPolygon(
                    c['poly'], facecolor='pink', edgecolor='lime',
                    alpha=1, label="detected pedestrian", ax=ax,
                    heading=True, hcolor='lime')
            else:
                helper.plotPolygon(
                    c['poly'], facecolor='pink', edgecolor='r',
                    alpha=1, label="undetected pedestrian", ax=ax,
                    heading=True, hcolor='r')

    def plotVehicle(self, ax):
        # plot other vehicle
        vehicleList = self.core.exportCurrentVehicle()
        hypoList = self.core.exportHypoVehicle()

        for hypo in hypoList:
            hp = hypo['p']
            a = 1
            b = len(hp)
            for i in range(0, b, int(b/5)):
                hpp = hp[i]
                a = max(a-0.15, 0.1)
                pose = hpp['pos']
                lw = hpp['std']
                helper.plotEllipse(
                    x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                    facecolor='pink', edgecolor='tab:pink', alpha=a, ax=ax)

            hc = hypo['c']
            helper.plotPolygon(
                    hc['poly'], facecolor='yellow', edgecolor='k',
                    alpha=1, label="hypothesis vehicle", ax=ax,
                    heading=True, hcolor='k')

        for veh in vehicleList:
            c = veh['c']
            p = veh['p']

            a = 1
            b = len(p)
            for i in range(0, b, int(b/5)):
                pp = p[i]
                a = max(a-0.15, 0.1)
                pose = pp['pos']
                lw = pp['std']
                if c['visible']:
                    helper.plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='lime', edgecolor='tab:pink', alpha=a, ax=ax)
                else:
                    helper.plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='lightcoral', edgecolor='tab:pink', alpha=a, ax=ax)

            if c['visible']:
                helper.plotPolygon(
                    c['poly'], facecolor='yellow', edgecolor='lime',
                    alpha=1, label="detected vehicle", ax=ax,
                    heading=True, hcolor='lime')
            else:
                helper.plotPolygon(
                    c['poly'], facecolor='yellow', edgecolor='r',
                    alpha=1, label="undetected vehicle", ax=ax,
                    heading=True, hcolor='r')

    def plotRoad(self, ax):
        # plot road
        for road in self.core._env._l_road:
            # draw left boundary
            helper.plotLine(
                road.left, ax=ax, color='k', linestyle='-')
            # draw right boundary
            helper.plotLine(
                road.right, ax=ax, color='k', linestyle='-')
            # draw lane in dashed
            helper.plotLine(
                road.lane, ax=ax, color='gray', linestyle='--')

    def plotFOV(self, ax):
        # plot FOV
        helper.plotPolygon(
            self.core.getCurrentFOV(), facecolor='gainsboro', edgecolor='lightcoral',
            alpha=0.7, ax=ax, label="FOV")

    def plotStaticObject(self, ax):
        # plot static object
        objectList = self.core._env._l_staticObject
        for obj in objectList:
            helper.plotPolygon(
                obj._poly, facecolor='gray', edgecolor='gray',
                alpha=1, label=None, ax=ax)

    def plotPedestrianCross(self, ax):
        # plot pedestrian cross
        crossList = self.core._env._l_cross
        for cross in crossList:
            v_l = cross.left[1] - cross.left[0]
            length = np.linalg.norm(v_l)
            v_l = v_l / length
            v_r = cross.right[1] - cross.right[0]
            v_r = v_r / length
            step = np.arange(0, length, 1)
            for i in range(0, step.shape[0]-1, 2):
                p1 = cross.left[0] + v_l * step[i]
                p2 = cross.right[0] + v_r * step[i]
                p3 = cross.right[0] + v_r * step[i+1]
                p4 = cross.left[0] + v_l * step[i+1]
                cPoly = np.array([p1, p2, p3, p4])
                helper.plotPolygon(
                    cPoly, facecolor='k', edgecolor='k',
                    alpha=0.5, label=None, ax=ax,
                    )

    def plotEgoVehicle(self, ax):
        # plot ego vehicle
        helper.plotPolygon(
            self.core.getCurrentEgoPoly(), facecolor='b', edgecolor='b',
            alpha=1, label='ego vehicle', ax=ax,
            heading=True, hcolor='k')


