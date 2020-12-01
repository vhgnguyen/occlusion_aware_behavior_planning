from PyQt5.QtWidgets import (QVBoxLayout, QGroupBox, QTextEdit,
                             QLineEdit, QGridLayout, QOpenGLWidget,
                             QLabel, QPushButton)
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QPolygon
from PyQt5.QtCore import QSize, Qt, QPoint

import OpenGL.GL as gl
import numpy as np
import helper

class BirdEyeView(QOpenGLWidget):

    def __init__(self, core, parent=None):
        super(BirdEyeView, self).__init__(parent)
        self.core = core

        # widget size
        self._width = 700
        self._height = 700
        self.setMaximumSize(self._width, self._height)
        self.setMinimumSize(self._width, self._height)

        # draw prediction
        self._drawPredict = False
        self._drawPredictHypo = False
        self._drawPredictObject = False

        # world coordinate
        self._rulerScale = 5
        self._size_x = 100
        self._size_y = 100
        self._x_center = -10
        self._y_center = 0
        self.center = False
        self._x_center_fix = 0
        self._y_center_fix = 0

        # background color
        self.trolltechPurple = QColor.fromCmykF(0.29, 0.29, 0.0, 0.0)

    def initializeGL(self):
        print(self.getOpenglInfo())
        self.setClearColor(self.trolltechPurple.darker())
        # gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glShadeModel(gl.GL_FLAT)
        gl.glEnable(gl.GL_LINE_SMOOTH)
        gl.glHint(gl.GL_LINE_SMOOTH_HINT, gl.GL_NICEST)
        gl.glEnable(gl.GL_POLYGON_SMOOTH)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glEnable(gl.GL_TEXTURE_2D)

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

    def setViewSize(self, x, y):
        self._size_x = x
        self._size_y = y
        self.update()

    def wheelEvent(self, event):
        d = int(event.angleDelta().y() / 120 * 10)
        dx = self._size_x - d
        dy = self._size_y - d
        if dx <= 0 or dy <= 0:
            return
        self._size_x = min(dx, 200)
        self._size_y = min(dy, 200)
        self.update()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_A:
            self._x_center_fix -= 10
        elif key == Qt.Key_W:
            self._y_center_fix += 10
        elif key == Qt.Key_D:
            self._x_center_fix += 10
        elif key == Qt.Key_S:
            self._y_center_fix -= 10
        self.update()

    def minimumSizeHint(self):
        return QSize(100, 100)

    def sizeHint(self):
        return QSize(900, 900)

    def paintGL(self):
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        if self.center:
            gl.glOrtho(
                self._x_center - self._size_x // 2, self._x_center + self._size_x // 2, 
                self._y_center - self._size_y // 2, self._y_center + self._size_y // 2,
                -1.0, 1.0)
        else:
            gl.glOrtho(
                self._x_center_fix - self._size_x // 2, self._x_center_fix + self._size_x // 2, 
                self._y_center_fix - self._size_y // 2, self._y_center_fix + self._size_y // 2,
                -1.0, 1.0)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        self.setClearColor(self.trolltechPurple.darker())
        gl.glClear(
            gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glLoadIdentity()
        self.makeObject()
        self.drawAxis()
        self.drawLegend()

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return
        gl.glViewport((width - side) // 2, (height - side) // 2, side, side)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        if self.center:
            gl.glOrtho(
                self._x_center - self._size_x // 2, self._x_center + self._size_x // 2, 
                self._y_center - self._size_y // 2, self._y_center + self._size_y // 2,
                -1.0, 1.0)
        else:
            gl.glOrtho(
                self._x_center_fix - self._size_x // 2, self._x_center_fix + self._size_x // 2, 
                self._y_center_fix - self._size_y // 2, self._y_center_fix + self._size_y // 2,
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
        for road in self.core._env._l_road:
            # draw left boundary
            helper.drawLine(
                line=road.left, color='black', alpha=1, lineWidth=4)
            # draw right boundary
            helper.drawLine(
                line=road.right, color='black', alpha=1, lineWidth=4)
            # draw lane in dashed
            helper.drawLine(
                line=road.lane, color='gray', alpha=1, lineWidth=2, strip=True)

    def drawStaticObject(self):
        objectList = self.core._env._l_staticObject
        for obj in objectList:
            helper.drawPoly(obj._poly, color='gray', alpha=0.5)

    def drawPedestrianCross(self):
        crossList = self.core._env._l_cross
        gl.glColor4f(0.8, 0.8, 0.8, 0.3)  # off white
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)
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
        pedesList = self.core.exportCurrentPedestrian()
        hypoList = self.core.exportHypoPedestrian()

        for hypo in hypoList:
            hc = hypo['c']
            helper.drawPoly(hc['poly'], color='pink', alpha=0.3)
            helper.drawHeading(hc['poly'], color='black',alpha=0.3)

            if self._drawPredictHypo:
                hp = hypo['p']
                for i in range(0, len(hp), 2):
                    hpp = hp[i]
                    helper.drawPoly(hpp['poly'], color='pink', alpha=0.1, fill=False)    

        for pedes in pedesList:
            c = pedes['c']
            c_color = 'lightGreen'
            if c['visible']:
                helper.drawPoly(c['poly'], color=c_color, alpha=0.5)
                helper.drawHeading(c['poly'], color='black', alpha=0.5)
            else:
                c_color = 'lightRed'
                helper.drawPoly(c['poly'], color=c_color, alpha=0.5)
                helper.drawHeading(c['poly'], color='black', alpha=0.5)

            if self._drawPredictObject:
                p = pedes['p']
                for i in range(0, len(p), ):
                    pp = p[i]
                    helper.drawPoly(pp['poly'], color=c_color, alpha=0.1, fill=False)    

    def drawOtherVehicle(self):
        vehicleList = self.core.exportCurrentVehicle()
        hypoList = self.core.exportHypoVehicle()

        for hypo in hypoList:
            hc = hypo['c']
            helper.drawPoly(hc['poly'], color='yellow', alpha=0.3)
            helper.drawHeading(hc['poly'], color='black',alpha=0.3)

            if self._drawPredictHypo:
                hp = hypo['p']
                for i in range(0, len(hp), 2):
                    hpp = hp[i]
                    helper.drawPoly(hpp['poly'], color='yellow', alpha=0.1, fill=False)  

        for vehicle in vehicleList:
            c = vehicle['c']
            c_color = 'green'
            if c['visible']:
                helper.drawPoly(c['poly'], color='green', alpha=0.5)
                helper.drawHeading(c['poly'], color='black', alpha=0.5)
            else:
                c_color = 'red'
                helper.drawPoly(c['poly'], color='red', alpha=0.5)
                helper.drawHeading(c['poly'], color='black', alpha=0.5)

            if self._drawPredictObject:
                p = vehicle['p']
                for i in range(0, len(p), 2):
                    pp = p[i]
                    helper.drawPoly(pp['poly'], color=c_color, alpha=0.1, fill=False)   

    def drawEgoVehicle(self):
        egoPoly = self.core.getCurrentEgoPoly()
        pos = self.core.getCurrentEgoPos()
        if pos is not None:
            c = 'black'
            if self.core.getCurrentAcceleration() > 0.3:
                c = 'green'
            if self.core.getCurrentAcceleration() < -0.3:
                c = 'red'
            helper.drawPoint(self.core.getCurrentPath(), color='lightRed')

            helper.drawPoly(egoPoly, color='blue', alpha=1)
            helper.drawHeading(egoPoly, color=c, alpha=1)

            self._x_center = pos[0]
            self._y_center = pos[1]

            p_pose = self.core.getPredictEgo()
            if self._drawPredict:
                for i in range(0, len(p_pose), 2):
                    pp = p_pose[i]
                    helper.drawPoly(pp['poly'], color='blue', alpha=0.1, fill=False)  

    def drawFOV(self):
        l_FOV = self.core.getCurrentFOV()
        helper.drawPoly(l_FOV, color='lightBlue', alpha=0.3, fill=False)

    def drawAxis(self):
        if self.center:
            xc = self._x_center
            yc = self._y_center
        else:
            xc = self._x_center_fix
            yc = self._y_center_fix
        xAxisMin = xc - self._size_x // 2
        xAxisMax = xc + self._size_x // 2
        startX = int(xAxisMin / self._rulerScale)
        endX = int(xAxisMax / self._rulerScale)

        yAxisMin = yc - self._size_y // 2
        yAxisMax = yc + self._size_y // 2
        startY = int(yAxisMin / self._rulerScale)
        endY = int(yAxisMax / self._rulerScale)

        painter = QPainter()
        painter.begin(self)
        color = QColor(Qt.white)
        color.setAlpha(200)
        painter.setPen(QPen(color))
        font = painter.font()
        font.setPointSize(font.pointSize() / 1.5)
        painter.setFont(font)

        for i in range(startX, endX + 1, 1):
            posX = self._rulerScale * i
            x = (posX - xAxisMin) / self._size_x * self._width
            painter.drawText(x - 10, self._height - 10, str(posX))
            painter.drawLine(x, self._height, x, self._height - 5)
        for i in range(startY, endY + 1, 1):
            posY = self._rulerScale * i
            y = (yAxisMax - posY) / self._size_y * self._height
            painter.drawText(10, y, str(posY))
            painter.drawLine(0, y, 5, y)
        painter.end()

    def drawLegend(self):
        self.drawObject(40, 10, QColor(Qt.blue), "Ego vehicle")
        self.drawObject(40, 40, QColor(Qt.yellow), "Hypothetical vehicle")
        self.drawObject(40, 70, QColor(255, 140, 140), "Hypothetical pedestrian")
        self.drawInfo(400, 30, name1="v=  ", name2=str(self.core.getCurrentVelocity())+ " m/s")
        a = self.core.getCurrentAcceleration()
        c = QColor(Qt.red)
        if a is not None and a > 0:
            c = QColor(Qt.green)
        self.drawInfo(400, 60, name1="a=  ", name2=str(a)+ " m/s2", color2=c)
        state = self.core.getCurrentState()
        if state == "Emergency":
            textColor = QColor(Qt.red)
        else:
            textColor = QColor(Qt.green)
        self.drawInfo(400, 90, name1=state, textColor=textColor)

    def drawObject(self, x, y, color, name):
        textColor = QColor(Qt.white)
        textColor.setAlpha(200)
        # draw polygon
        painter = QPainter()
        painter.begin(self)
        painter.setBrush(QBrush(color))
        points = QPolygon([QPoint(x, y), QPoint(x+50, y), QPoint(x+50, y+20), QPoint(x, y+20)])
        painter.drawPolygon(points)
        painter.end()
        # draw text
        painter.begin(self)
        painter.setPen(QPen(textColor))
        font = painter.font()
        font.setPointSize(font.pointSize())
        painter.setFont(font)
        painter.drawText(x+70, y+20, name)
        painter.end()
    
    def drawInfo(self, x, y, name1, name2=None, textColor=QColor(Qt.white), color2=QColor(Qt.white)):
        textColor.setAlpha(200)
        painter = QPainter()

        painter.begin(self)
        painter.setPen(QPen(textColor))
        font = painter.font()
        font.setPointSize(font.pointSize())
        painter.setFont(font)
        painter.drawText(x, y, name1)
        painter.end()

        if name2 is not None:
            painter.begin(self)
            color2.setAlpha(200)
            painter.setPen(QPen(color2))
            font = painter.font()
            font.setPointSize(font.pointSize())
            painter.setFont(font)
            painter.drawText(x+30, y, name2)
            painter.end()

    def setClearColor(self, c):
        gl.glClearColor(c.redF(), c.greenF(), c.blueF(), 0)

    def setColor(self, c):
        gl.glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF())


class LegendWidget(QOpenGLWidget):

    def __init__(self, parent=None):
        super(LegendWidget, self).__init__(parent)
        # widget size
        self._width = 200
        self._height = 100
        self.setMaximumSize(self._width, self._height)
        self.setMinimumSize(self._width, self._height)
        self.trolltechPurple = QColor.fromCmykF(0.29, 0.29, 0.0, 0.0)

    def initializeGL(self):
        self.setClearColor(self.trolltechPurple.darker())

        gl.glShadeModel(gl.GL_FLAT)
        # gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_POLYGON_SMOOTH)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

    def setClearColor(self, c):
        gl.glClearColor(c.redF(), c.greenF(), c.blueF(), 0)

    def setColor(self, c):
        gl.glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    # def drawLegend(self):
