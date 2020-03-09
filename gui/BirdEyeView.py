from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)
from PyQt5.QtGui import QColor
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

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


class BirdEyeView(QOpenGLWidget):

    def __init__(self, parent=None):
        super(BirdEyeView, self).__init__(parent)
        self.setMaximumSize(1800, 900)
        self.setMinimumSize(900, 450)

    def initializeGL(self):
        print(self.getOpenglInfo())
        self.setClearColor(QColor.fromCmykF(0.3, 0.3, 0.0, 0.0).darker())
        glShadeModel(GL_FLAT)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        self.resizeGL(5, 5)
        self.drawRoadBoundary(1)

    def setClearColor(self, c):
        glClearColor(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    def getOpenglInfo(self):
        info = """
            Vendor: {0}
            Renderer: {1}
            OpenGL Version: {2}
            Shader Version: {3}
        """.format(
            glGetString(GL_VENDOR),
            glGetString(GL_RENDERER),
            glGetString(GL_VERSION),
            glGetString(GL_SHADING_LANGUAGE_VERSION)
        )
        return info
    
    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        glViewport((width - side) // 2, (height - side) // 2, side, side)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0)
        glMatrixMode(GL_MODELVIEW)

    def drawRoadBoundary(self, road):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glColor3f(0.0, 0.0, 1.0)
        print("Drawing line")
        glBegin(GL_LINES)
        glVertex2f(-5.0, -5.0)
        glVertex2f(5.0, 5.0)
        glEnd()

    def drawStaticObject(self, obs):
        return None

    def drawDynamicObject(self, obs):
        return None

    def drawEgoVehicle(self, ego):
        return None
