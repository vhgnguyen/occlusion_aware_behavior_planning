import OpenGL.GL as gl
from enum import Enum


def setColor(color, alpha):
    if color == 'red':
        return gl.glColor4f(1.0, 0.0, 0.0, alpha)
    if color == 'lightRed':
        return gl.glColor4f(1.0, 0.1, 0.5, alpha)
    if color == 'green':
        return gl.glColor4f(0.0, 1.0 , 0.0, alpha)
    if color == 'lightGreen':
        return gl.glColor4f(0.5, 1.0, 0.1, alpha)
    if color == 'blue':
        return gl.glColor4f(0.0, 0.0, 1.0, alpha)
    if color == 'yellow':
        return gl.glColor4f(1.0, 1.0, 0.0, alpha)
    if color == 'white':
        return gl.glColor4f(1.0, 1.0, 1.0, alpha)
    if color == 'black':
        return gl.glColor4f(0.0, 0.0, 0.0, alpha)
    if color == 'gray':
        return gl.glColor4f(0.5, 0.5, 0.5, alpha)
    if color == 'pink':
        return gl.glColor4f(1.0, 0.6, 0.6, alpha)
    if color == 'lightBlue':
        return gl.glColor4f(0.0, 0.8, 0.8, alpha)


def drawPoly(poly, color, alpha, fill=True):
    if poly is None:
        return
    setColor(color=color, alpha=alpha)
    if poly.shape[0] == 4:
        gl.glBegin(gl.GL_QUADS)
        for p in poly:
            gl.glVertex2f(p[0], p[1])
        gl.glEnd()
    else:
        if fill:
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)
        else:
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glBegin(gl.GL_POLYGON)
        for p in poly:
            gl.glVertex2f(p[0], p[1])
        gl.glEnd()
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)


def drawLine(line, color, alpha, lineWidth, strip=False):
    if line is None or line.shape[0] != 2:
        return
    setColor(color=color, alpha=alpha)
    gl.glLineWidth(lineWidth)
    if strip:
        gl.glLineStipple(1, 0xAAAA)
        gl.glEnable(gl.GL_LINE_STIPPLE)
        gl.glBegin(gl.GL_LINES)
        gl.glVertex2f(line[0][0], line[0][1])
        gl.glVertex2f(line[1][0], line[1][1])
        gl.glEnd()
        gl.glDisable(gl.GL_LINE_STIPPLE)
    else:
        gl.glBegin(gl.GL_LINES)
        gl.glVertex2f(line[0][0], line[0][1])
        gl.glVertex2f(line[1][0], line[1][1])
        gl.glEnd()
