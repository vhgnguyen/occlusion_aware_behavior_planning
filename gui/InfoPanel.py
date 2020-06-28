from PyQt5 import QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QGroupBox,
                             QVBoxLayout, QHBoxLayout, QPushButton,
                             QLineEdit, QGridLayout, QLabel, QCheckBox)
import numpy as np
import matplotlib.pyplot as plt
import tikzplotlib
from matplotlib.patches import Ellipse, Polygon
import helper as helper
from helper import PlotScene


class InfoPanel(QGroupBox):

    def __init__(self, core, parent=None):
        super(InfoPanel, self).__init__(parent)
        self.core = core
        self.plot = PlotScene(core=core)
        self.setTitle("Information panel")
        self.setMaximumSize(400, 900)
        self.setMinimumSize(400, 900)
        self.mainLayout = QVBoxLayout()
        self.addInfoBox()
        self.addGraphBox()
        self.addPlotBox()
        self.mainLayout.addStretch()
        self.setLayout(self.mainLayout)

    def addInfoBox(self):
        # information control
        self.infoBox = QGroupBox("Information")
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

        self.infoGrid.addWidget(QLabel("Travel distance"), 4, 0)
        self.egoDisValue = QLabel()
        self.egoDisValue.setMaximumWidth(100)
        self.infoGrid.addWidget(self.egoDisValue, 4, 1)

        self.infoBox.setLayout(self.infoGrid)
        self.mainLayout.addWidget(self.infoBox)
    
    def addGraphBox(self):
        # plot button
        self.graphBox = QGroupBox("Vehicle profile graph")
        self.graphGrid = QGridLayout()

        self.pathName = QLineEdit()
        self.pathName.setText('result/it/')
        self.pathName.setMaximumWidth(400)
        self.graphGrid.addWidget(self.pathName, 0, 0)

        self.plotDButton = QPushButton("Show dynamic plot")
        self.plotDButton.clicked.connect(self.on_plotD_clicked)
        self.plotDButton.setMaximumWidth(300)
        self.graphGrid.addWidget(self.plotDButton, 1, 0)

        self.safeV = QCheckBox("Safe velocity")
        self.safeV.setChecked(True)
        self.graphGrid.addWidget(self.safeV, 1, 1)

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

    def addPlotBox(self):
        self.plotBox = QGroupBox("Plot scene")
        self.plotGrid = QGridLayout()
        
        self.plotPoseCheck = QCheckBox("Plot ego vehicle at center")
        self.plotPoseCheck.setChecked(True)
        self.plotGrid.addWidget(self.plotPoseCheck, 0, 0)

        self.plotGrid.addWidget(QLabel("Center [x, y]"), 1, 0)
        self.plotX = QLineEdit()
        self.plotX.setText(str(-30))
        self.plotX.setMaximumWidth(100)
        self.plotGrid.addWidget(self.plotX, 1, 1)

        self.plotY = QLineEdit()
        self.plotY.setText(str(0))
        self.plotY.setMaximumWidth(100)
        self.plotGrid.addWidget(self.plotY, 1, 2)

        self.plotGrid.addWidget(QLabel("Plot size [h, w]"), 2, 0)
        self.plotH = QLineEdit()
        self.plotH.setText(str(20))
        self.plotH.setMaximumWidth(100)
        self.plotGrid.addWidget(self.plotH, 2, 1)

        self.plotW = QLineEdit()
        self.plotW.setText(str(100))
        self.plotW.setMaximumWidth(100)
        self.plotGrid.addWidget(self.plotW, 2, 2)

        self.plotSceneButton = QPushButton("Show scene plot")
        self.plotSceneButton.clicked.connect(self.on_plotScene_clicked)
        self.plotSceneButton.setMaximumWidth(300)
        self.plotGrid.addWidget(self.plotSceneButton, 3, 0)

        self.plotGrid.addWidget(QLabel("Tikz size [h, w]"), 4, 0)
        self.tikzH = QLineEdit()
        self.tikzH.setText(str(5))
        self.tikzH.setMaximumWidth(100)
        self.plotGrid.addWidget(self.tikzH, 4, 1)

        self.tikzW = QLineEdit()
        self.tikzW.setText(str(12))
        self.tikzW.setMaximumWidth(100)
        self.plotGrid.addWidget(self.tikzW, 4, 2)

        self.saveSceneName = QLineEdit()
        self.saveSceneName.setText("scene.tex")
        self.saveSceneName.setMaximumWidth(400)
        self.plotGrid.addWidget(self.saveSceneName, 5, 0, 1, 2)

        self.saveSceneButton = QPushButton("Save")
        self.saveSceneButton.clicked.connect(self.on_saveScene_clicked)
        self.saveSceneButton.setMaximumWidth(100)
        self.plotGrid.addWidget(self.saveSceneButton, 5, 2)
        
        self.plotBox.setLayout(self.plotGrid)
        self.mainLayout.addWidget(self.plotBox)

    def update(self):
        if self.core._egoCar is not None:
            self.currentTimeValue.setText(str(self.core.getCurrentTime()) + " s")
            self.simulationTimeValue.setText(str(self.core.getSimulationTime()) + " s")
            self.egoVelocityValue.setText(str(round(self.core.getCurrentVelocity(), 2)) + " m/s")
            self.egoAccValue.setText(str(round(self.core.getCurrentAcceleration(), 2)) + " m/s<sup>2</sup>")
            self.egoDisValue.setText(str(round(self.core.getTravelDistance(), 2)) + " m")

    def on_plotD_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotDynamic(safeVelocity=self.safeV.isChecked())

    def on_plotR_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotRisk()

    def on_plotDD_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotDynamicDistance()

    def on_plotScene_clicked(self):
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
                axis_height=self.tikzH.text()+'cm',
                axis_width=self.tikzW.text()+'cm')

    def plotScene(self):
        x = float(self.plotX.text())
        y = float(self.plotY.text())
        h = float(self.plotH.text())
        w = float(self.plotW.text())
        self.plot.setCoordinate(
            x=x,y=y,h=h,w=w,
            poseCenter=self.plotPoseCheck.isChecked()
        )
        figH = int(self.tikzH.text())
        figW = int(self.tikzW.text())
        self.plot.plotScene(h=figH, w=figW)