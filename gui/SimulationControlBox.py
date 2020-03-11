from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)

class SimulationControlBox(QGroupBox):

    def __init__(self, core, parent=None):
        super(SimulationControlBox, self).__init__(parent)

        self.core = core

        self.setTitle("Simulation control")
        # self.setMaximumSize(500, 400)
        self.setMinimumSize(250, 200)
        self.simulationGrid = QGridLayout()
        self.simulationGrid.setSpacing(10)

        self.simulationGrid.addWidget(QLabel("Simulation time [s]"), 0, 0)
        self.simulationGrid.addWidget(QLabel("Current simulation timestamp [s]"), 1, 0)

        self.simulationTimeValue = QLabel()
        self.simulationTimeValue.setMaximumWidth(100)
        self.currentTimeValue = QLabel()
        self.simulationGrid.addWidget(self.simulationTimeValue, 0, 1)
        self.simulationGrid.addWidget(self.currentTimeValue, 1, 1)
        self.setLayout(self.simulationGrid)

    def update(self):
        self.currentTimeValue.setText(str(self.core.getCurrentTime()))
        self.simulationTimeValue.setText(str(self.core.getSimulationTime()))

    def generateValuesButtonClicked(self, simulation_time):
        return None
