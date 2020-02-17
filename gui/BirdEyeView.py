from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)
from PyQt5.QtGui import QColor
import OpenGL.GL as gl


class SimulationControlBox(QGroupBox):

    def __init__(self, parent=None):
        super(SimulationControlBox, self).__init__(parent)
        self.setTitle("Simulation control")
        self.setMaximumSize(1800, 900)
        self.setMinimumSize(900, 600)
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


class BirdEyeView(QOpenGLWidget):

    def __init__(self, parent=None):
        super(BirdEyeView, self).__init__(parent)
        self.setMaximumSize(1800, 900)
        self.setMinimumSize(900, 600)

    def initializeGL(self):
        print(self.getOpenglInfo())
        self.setClearColor(QColor.fromCmykF(0.3, 0.3, 0.0, 0.0).darker())
        gl.glShadeModel(gl.GL_FLAT)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_CULL_FACE)

    def setClearColor(self, c):
        gl.glClearColor(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    def getOpenglInfo(self):
        info = """
            Vendor: {0}
            Renderer: {1}
            OpenGL Version: {2}
            Shader Version: {3}
        """.format(
            gl.glGetString(gl.GL_VENDOR),
            gl.glGetString(gl.GL_RENDERER),
            gl.glGetString(gl.GL_VERSION),
            gl.glGetString(gl.GL_SHADING_LANGUAGE_VERSION)
        )
        return info

    def drawRoadBoundary(self, road):
        return None

    def drawStaticObject(self, obs):
        return None

    def drawDynamicObject(self, obs):
        return None

    def drawEgoVehicle(self, ego):
        return None
