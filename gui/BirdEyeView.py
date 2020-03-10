from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)
from PyQt5.QtGui import QColor
from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
import OpenGL.GL as gl
import math
import numpy as np


class BirdEyeView(QOpenGLWidget):

    def __init__(self, core, parent=None):
        super(BirdEyeView, self).__init__(parent)

        self.core = core

        self.trolltechGreen = QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)

    def initializeGL(self):
        print(self.getOpenglInfo())

        self.setClearColor(self.trolltechPurple.darker())
        self.object = self.makeObject()

        gl.glShadeModel(gl.GL_FLAT)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_CULL_FACE)

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
        return QSize(50, 50)

    def sizeHint(self):
        return QSize(400, 400)

    def paintGL(self):
        gl.glClear(
            gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()
        gl.glTranslated(0.0, 0.0, -10.0)
        self.makeObject()

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return
        gl.glViewport((width - side) // 2, (height - side) // 2, side, side)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(-50, 50, -50, 50, 4.0, 15.0)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

    def makeObject(self):
        self.drawRoadBoundary(self.core._env._l_road)
        self.drawStaticObject(self.core._env._l_staticObject)

    def drawRoadBoundary(self, road):
        gl.glColor4f(0.6, 0.6, 0.6, 1.0)  # gray
        gl.glLineWidth(2.0)

        if road == 0:
            return

        # draw left boundary
        gl.glBegin(gl.GL_LINES)
        for l in road.left:
            gl.glVertex2f(l[0][0], l[0][1])
            gl.glVertex2f(l[1][0], l[1][1])
        gl.glEnd()

        # # draw right boundary
        gl.glBegin(gl.GL_LINES)
        for l in road.right:
            gl.glVertex2f(l[0][0], l[0][1])
            gl.glVertex2f(l[1][0], l[1][1])
        gl.glEnd()

        # # draw lane in dashed
        gl.glLineStipple(1, 0xAAAA)  # [1]
        gl.glEnable(gl.GL_LINE_STIPPLE)
        gl.glBegin(gl.GL_LINES)
        for l in road.lane:
            gl.glVertex2f(l[0][0], l[0][1])
            gl.glVertex2f(l[1][0], l[1][1])
        gl.glEnd()
        gl.glDisable(gl.GL_LINE_STIPPLE)

    def drawStaticObject(self, objectList):
        gl.glColor4f(0.0, 0.0, 0.0, 1.0)  # black

        # poly should be sorted in counter clock-wise
        for obj in objectList:
            gl.glBegin(gl.GL_POLYGON)
            for i, pt in enumerate(obj._poly):
                gl.glVertex2f(pt[0], pt[1])
            gl.glEnd()

    def drawPedestrian(self, pedestrianPoly):
        return None

    def drawOtherVehicle(self, vehiclePoly):
        return None

    def drawEgoVehicle(self, egoPoly):
        return None

    def setClearColor(self, c):
        gl.glClearColor(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    def setColor(self, c):
        gl.glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF())
