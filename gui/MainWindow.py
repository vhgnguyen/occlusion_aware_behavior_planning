from PyQt5 import QtCore
from PyQt5.QtWidgets import (QMainWindow, QWidget, QGroupBox,
                             QVBoxLayout, QHBoxLayout, QPushButton,
                             QLineEdit, QGridLayout)

from BirdEyeView import BirdEyeView
from SimulationControlBox import SimulationControlBox
from InputPanel import InputWidget
import time

import _param as param

class MainWindow(QMainWindow):

    def __init__(self, core, parent=None):
        super(MainWindow, self).__init__(parent)
        self.core = core
        self.setGeometry(50, 50, 1000, 1000)

        # timer
        self.plots_refresh_timer = None

        # self.setMaximumSize(1920, 1080)
        self.setMinimumSize(1400, 1000)
        self.setWindowTitle("Risk Estimation GUI")
        self.widget = QWidget(self)
        self.setCentralWidget(self.widget)
        self.mainLayout = QHBoxLayout()
        self.initUI()
        self.show()

    def initUI(self):
        """ Simulation layout """
        self.glLayout = QVBoxLayout()
        self.glLayout.setAlignment(QtCore.Qt.AlignHCenter)

        # bird eye view
        self.birdEyeView = BirdEyeView(self.core)
        self.glLayout.addWidget(self.birdEyeView)

        # Button box
        self.buttonBox = QGroupBox()
        self.buttonBox.setTitle("Control buttons")
        self.buttonGrid = QGridLayout()

        # refresh view button
        self.refreshButton = QPushButton("Refresh")
        self.refreshButton.setMaximumWidth(200)
        self.refreshButton.clicked.connect(self.on_refreshButton_clicked)
        self.buttonGrid.addWidget(self.refreshButton, 0, 0)
        
        # move button
        self.moveButton = QPushButton("Next state")
        self.moveButton.clicked.connect(self.on_moveButton_clicked)
        self.buttonGrid.addWidget(self.moveButton, 1, 1)
        # simulation button
        self.simulationButton = QPushButton("Start simulation")
        self.simulationButton.clicked.connect(self.on_simulationButton_clicked)
        self.buttonGrid.addWidget(self.simulationButton, 1, 0)
        # stop button
        self.stopSimulationButton = QPushButton("Pause simulation")
        self.stopSimulationButton.clicked.connect(self.on_stopSimulation_clicked)
        self.buttonGrid.addWidget(self.stopSimulationButton, 1, 2)

        self.buttonBox.setLayout(self.buttonGrid)
        self.glLayout.addWidget(self.buttonBox)

        """ control box """
        self.simulationControlBox = SimulationControlBox(self.core)
        self.glLayout.addWidget(self.simulationControlBox)

        """ Input layout """
        self.inputLayout = QVBoxLayout()
        self.inputLayout.setAlignment(QtCore.Qt.AlignTop)
        self.inputWidget = InputWidget(self.core)
        self.inputLayout.addWidget(self.inputWidget)

        # Main layout
        self.mainLayout.addLayout(self.inputLayout)
        self.mainLayout.addLayout(self.glLayout)
        self.widget.setLayout(self.mainLayout)

    def on_refreshButton_clicked(self):
        self.birdEyeView.update()
        self.simulationControlBox.update()

    def on_moveButton_clicked(self):
        self.core.move()
        self.birdEyeView.update()
        self.simulationControlBox.update()

    def on_simulationButton_clicked(self):
        self.plots_refresh_timer = QtCore.QTimer()
        self.plots_refresh_timer.setSingleShot(False)
        self.plots_refresh_timer.timeout.connect(self.on_simulation)
        self.plots_refresh_timer.setInterval(99)
        self.plots_refresh_timer.start()

    def on_stopSimulation_clicked(self):
        if self.plots_refresh_timer is not None:
            self.plots_refresh_timer.stop()

    def on_simulation(self):
        isMove = self.core.move()
        if isMove:
            self.birdEyeView.update()
            self.simulationControlBox.update()
        else:
            self.plots_refresh_timer.stop()

    def keyPressEvent(self, event):
        self.birdEyeView.keyPressEvent(event)
