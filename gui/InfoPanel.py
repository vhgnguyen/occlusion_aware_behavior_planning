from PyQt5 import QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QGroupBox,
                             QVBoxLayout, QHBoxLayout, QPushButton,
                             QLineEdit, QGridLayout, QLabel)


class InfoPanel(QGroupBox):

    def __init__(self, core, parent=None):
        super(InfoPanel, self).__init__(parent)
        self.core = core
        self.setTitle("Information panel")
        self.setMaximumSize(500, 1080)
        self.setMinimumSize(500, 1080)
        self.mainLayout = QVBoxLayout()
        self.addInfoBox()
        self.mainLayout.addStretch()
        self.setLayout(self.mainLayout)

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

        self.plotButton1 = QPushButton("Show dynamic plot")
        self.plotButton1.clicked.connect(self.on_plot1_clicked)
        self.plotButton1.setMaximumWidth(200)
        self.graphGrid.addWidget(self.plotButton1, 2, 0)

        self.plotButton2 = QPushButton("Show risk plot")
        self.plotButton2.clicked.connect(self.on_plot2_clicked)
        self.plotButton2.setMaximumWidth(200)
        self.graphGrid.addWidget(self.plotButton2, 2, 1)

        self.graphBox.setLayout(self.graphGrid)
        self.mainLayout.addWidget(self.graphBox)

    def update(self):
        if self.core._egoCar is not None:
            self.currentTimeValue.setText(str(self.core.getCurrentTime()) + " s")
            self.simulationTimeValue.setText(str(self.core.getSimulationTime()) + " s")
            self.egoVelocityValue.setText(str(round(self.core.getCurrentVelocity(), 2)) + " m/s")
            self.egoAccValue.setText(str(round(self.core.getCurrentAcceleration(), 2)) + " m/s<sup>2</sup>")

    def on_plot1_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotDynamic()

    def on_plot2_clicked(self):
        if self.core._egoCar is not None:
            self.core.plotRisk()
