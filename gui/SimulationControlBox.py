from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)

class SimulationControlBox(QGroupBox):

    def __init__(self, core, parent=None):
        super(SimulationControlBox, self).__init__(parent)

        self.core = core

        self.setTitle("Information panel")
        # self.setMaximumSize(500, 400)
        self.setMinimumSize(250, 200)

        self.mainLayout = QVBoxLayout()

        self.addBox()

        self.mainLayout.addStretch()
        self.setLayout(self.mainLayout)

    def addBox(self):

        # simulation control
        self.simuBox = QGroupBox()
        self.simuBox.setTitle("Simulation control")
        self.simuGrid = QGridLayout()

        self.simuGrid.addWidget(QLabel("Simulation time [s]"), 0, 0)
        self.simulationTimeValue = QLabel()
        self.simulationTimeValue.setMaximumWidth(100)
        self.simuGrid.addWidget(self.simulationTimeValue, 0, 1)

        self.simuGrid.addWidget(QLabel("Current simulation timestamp [s]"), 1, 0)
        self.currentTimeValue = QLabel()
        self.simulationTimeValue.setMaximumWidth(100)
        self.simuGrid.addWidget(self.currentTimeValue, 1, 1)

        self.simuBox.setLayout(self.simuGrid)
        self.mainLayout.addWidget(self.simuBox)

        # ego control
        self.egoBox = QGroupBox()
        self.egoBox.setTitle("Ego state")
        self.egoGrid = QGridLayout()

        self.egoGrid.addWidget(QLabel("Velocity"), 0, 0)
        self.egoVelocityValue = QLabel()
        self.egoVelocityValue.setMaximumWidth(100)
        self.egoGrid.addWidget(self.egoVelocityValue, 0, 1)

        self.egoGrid.addWidget(QLabel("Acceleration"), 1, 0)
        self.egoAccValue = QLabel()
        self.egoAccValue.setMaximumWidth(100)
        self.egoGrid.addWidget(self.egoAccValue, 1, 1)

        # plot button
        self.plotButton = QPushButton("Show plot")
        self.plotButton.clicked.connect(self.on_plot_clicked)
        self.egoGrid.addWidget(self.plotButton, 2, 0)

        self.egoBox.setLayout(self.egoGrid)
        self.mainLayout.addWidget(self.egoBox)

    def update(self):
        if self.core._egoCar is not None:
            self.currentTimeValue.setText(str(self.core.getCurrentTime()) + " s")
            self.simulationTimeValue.setText(str(self.core.getSimulationTime()) + " s")
            self.egoVelocityValue.setText(str(round(self.core.getCurrentVelocity(), 2)) + " m/s")
            self.egoAccValue.setText(str(round(self.core.getCurrentAcceleration(), 2)) + " m/s<sup>2</sup>")

    def on_plot_clicked(self):
        if self.core._egoCar is not None:
            self.core.plot()
