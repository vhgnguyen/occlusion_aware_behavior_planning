from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
import OpenGL.GL as gl
import math
import numpy as np


class BirdEyeView(QOpenGLWidget):
    
    
    def __init__(self, core, parent=None):
        super(BirdEyeView, self).__init__(parent)

        self.core = core

        # widget size
        self._width = 800
        self._height = 600
        self.setMaximumSize(self._width * 2, self._height * 2)
        self.setMinimumSize(self._width, self._height)

        # world coordinate 
        self._rulerScale = 20
        self._sizeX = 100
        self._sizeY = 100
        self._x_center = -20
        self._y_center = -20

        self.trolltechPurple = QColor.fromCmykF(0.29, 0.29, 0.0, 0.0)

    def initializeGL(self):
        print(self.getOpenglInfo())

        self.setClearColor(self.trolltechPurple.darker())

        gl.glShadeModel(gl.GL_FLAT)
        # gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_POLYGON_SMOOTH)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

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

    def minimumSizeHint(self):
        return QSize(100, 100)

    def sizeHint(self):
        return QSize(900, 900)

    def paintGL(self):
        self.setClearColor(self.trolltechPurple.darker())
        gl.glClear(
            gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()
        self.drawAxis()
        self.makeObject()
        
    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return
        gl.glViewport((width - side) // 2, (height - side) // 2, side, side)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(
            self._x_center - self._sizeX // 2, self._x_center + self._sizeX // 2, 
            self._y_center - self._sizeY // 2, self._y_center + self._sizeY // 2,
            -1.0, 1.0)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        self._width = self.size().width()
        self._height = self.size().height()

    def makeObject(self):
        # draw static object
        self.drawPedestrianCross()
        self.drawRoadBoundary()
        self.drawStaticObject()

        # draw moving object
        self.drawEgoVehicle()
        self.drawOtherVehicle()
        self.drawPedestrian()
        self.drawFOV()

    def drawRoadBoundary(self):
        road = self.core._env._l_road
        if road == 0:
            return
        gl.glColor4f(0., 0., 0., 1.0)  # gray
        gl.glLineWidth(4.0)
        # draw left boundary
        gl.glBegin(gl.GL_LINES)
        for l in road.left:
            gl.glVertex2f(l[0][0], l[0][1])
            gl.glVertex2f(l[1][0], l[1][1])
        gl.glEnd()
        # draw right boundary
        gl.glBegin(gl.GL_LINES)
        for l in road.right:
            gl.glVertex2f(l[0][0], l[0][1])
            gl.glVertex2f(l[1][0], l[1][1])
        gl.glEnd()
        # draw lane in dashed
        gl.glColor4f(0.6, 0.6, 0.6, 1.0)  # gray
        gl.glLineWidth(2.0)
        gl.glLineStipple(1, 0xAAAA)  # [1]
        gl.glEnable(gl.GL_LINE_STIPPLE)
        gl.glBegin(gl.GL_LINES)
        for l in road.lane:
            gl.glVertex2f(l[0][0], l[0][1])
            gl.glVertex2f(l[1][0], l[1][1])
        gl.glEnd()
        gl.glDisable(gl.GL_LINE_STIPPLE)

    def drawStaticObject(self):
        objectList = self.core._env._l_staticObject
        gl.glColor4f(0.5, 0.5, 0.5, 0.8)  # black
        # poly should be sorted in counter clock-wise
        for obj in objectList:
            gl.glBegin(gl.GL_POLYGON)
            for i, pt in enumerate(obj._poly):
                gl.glVertex2f(pt[0], pt[1])
            gl.glEnd()

    def drawPedestrianCross(self):
        crossList = self.core._env._l_cross
        gl.glColor4f(0.8, 0.8, 0.8, 0.3)  # off white
        gl.glBegin(gl.GL_QUADS)
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
                gl.glVertex2f(p1[0], p1[1])
                gl.glVertex2f(p2[0], p2[1])
                gl.glVertex2f(p3[0], p3[1])
                gl.glVertex2f(p4[0], p4[1])
        gl.glEnd()

    def drawPedestrian(self):
        gl.glColor4f(0.0, 0.2, 0.8, 0.6)  # blue
        gl.glBegin(gl.GL_QUADS)
        pedestrianPoly = self.core.currentPedestrianPoly()
        for pedestrian in pedestrianPoly:
            for vertex in pedestrian:
                gl.glVertex2f(vertex[0], vertex[1])
        gl.glEnd()

    def drawOtherVehicle(self):
        gl.glColor4f(0.0, 0.8, 0.2, 0.6)  # green
        gl.glBegin(gl.GL_QUADS)
        vehiclePoly = self.core.currentVehiclePoly()
        for vehicle in vehiclePoly:
            for vertex in vehicle:
                gl.glVertex2f(vertex[0], vertex[1])
        gl.glEnd()

    def drawEgoVehicle(self):
        gl.glColor4f(0.8, 0.2, 0.2, 0.6)  # red
        gl.glBegin(gl.GL_QUADS)
        egoPoly = self.core.currentEgoPoly()
        if egoPoly is not None:
            for vertex in egoPoly:
                gl.glVertex2f(vertex[0], vertex[1])
        gl.glEnd()

    def drawFOV(self):
        vehPos = self.core.getCurrentEgoPos()
        l_FOV = self.core.getCurrentFOV()

        gl.glColor4f(0.0, 0.0, 0.5, 0.1)  # orange
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        if (vehPos is not None and l_FOV is not None):
            gl.glBegin(gl.GL_POLYGON)
            # gl.glBegin(gl.GL_LINES)
            for pt in l_FOV:
                # gl.glVertex2f(vehPos[0], vehPos[1])
                gl.glVertex2f(pt[0], pt[1])
            gl.glEnd()
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)

    def drawAxis(self):

        xAxisMin = self._x_center - self._sizeX // 2
        xAxisMax = self._x_center + self._sizeX // 2
        startX = int(xAxisMin / self._rulerScale)
        endX = int(xAxisMax / self._rulerScale)

        yAxisMin = self._y_center - self._sizeY // 2
        yAxisMax = self._y_center + self._sizeY // 2
        startY = int(yAxisMin / self._rulerScale)
        endY = int(yAxisMax / self._rulerScale)

        painter = QPainter()
        painter.begin(self)
        painter.setPen(QPen(Qt.white))
        font = painter.font()
        font.setPointSize(font.pointSize() / 1.5)
        painter.setFont(font)
        # draw x-Axis
        for i in range(startX, endX + 1, 1):
            posX = self._rulerScale * i
            x = (posX - xAxisMin) / self._sizeX * self._width
            painter.drawText(x - 10, self._height - 10, str(posX))
            painter.drawLine(x, self._height, x, self._height - 5)
        for i in range(startY, endY + 1, 1):
            posY = self._rulerScale * i
            y = (yAxisMax - posY) / self._sizeY * self._height
            painter.drawText(10, y, str(posY))
            painter.drawLine(0, y, 5, y)
        painter.end()

    def setClearColor(self, c):
        gl.glClearColor(c.redF(), c.greenF(), c.blueF(), 0)

    def setColor(self, c):
        gl.glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF())
