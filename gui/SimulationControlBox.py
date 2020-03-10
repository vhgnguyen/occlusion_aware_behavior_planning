from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)

class SimulationControlBox(QGroupBox):

    def __init__(self, parent=None):
        super(SimulationControlBox, self).__init__(parent)
        self.setTitle("Simulation control")
        self.setMaximumSize(1800, 900)
        self.setMinimumSize(900, 450)
        self.simulationGrid = QGridLayout()
        self.simulationGrid.setSpacing(10)
        self.simulationGrid.addWidget(QLabel("Simulation time [s]"), 0, 0)
        self.simulationGrid.addWidget(QLabel("Current simulation timestamp [s]"), 1, 0)
        self.simulationTimeValue = QLineEdit()
        self.simulationTimeValue.setMaximumWidth(100)
        self.currentTimeValue = QLabel()
        self.simulationGrid.addWidget(self.simulationTimeValue, 0, 1)
        self.simulationGrid.addWidget(self.currentTimeValue, 1, 1)
        self.setLayout(self.simulationGrid)

    def updateSimulationTime(self, simulation_time):
        return None

    def updateCurrentTime(self, current_time):
        return None

    def generateValuesButtonClicked(self, simulation_time):
        return None
