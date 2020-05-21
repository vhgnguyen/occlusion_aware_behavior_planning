import OpenGL.GL as gl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Polygon, Rectangle


"""
    OpenGL draw functions
"""


def setColor(color, alpha):
    if color == 'red':
        return gl.glColor4f(1.0, 0.0, 0.0, alpha)
    if color == 'lightRed':
        return gl.glColor4f(1.0, 0.1, 0.5, alpha)
    if color == 'green':
        return gl.glColor4f(0.0, 1.0, 0.0, alpha)
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


"""
    Matplotlib plot functions
"""


def plotPolygon(poly, facecolor, edgecolor, alpha, label=None, ax=plt,
                heading=False, hcolor='k'):
    p = Polygon(
        poly, facecolor=facecolor, edgecolor=edgecolor,
        alpha=alpha, label=label
        )
    ax.add_patch(p)
    if heading:
        triangle = 0.5 * np.array([poly[0] + poly[1],
                                   poly[1] + poly[2],
                                   poly[2] + poly[3]])
        t = Polygon(
            triangle, facecolor=hcolor, edgecolor=edgecolor,
            alpha=alpha
        )
        ax.add_patch(t)


def plotLine(line, ax=plt, **kwargs):
    ax.plot(line[:, 0], line[:, 1], **kwargs)


def handleLegend(ax=plt):
    handles, labels = ax.gca().get_legend_handles_labels()
    newLabels, newHandles = [], []
    for handle, label in zip(handles, labels):
        if label not in newLabels:
            newLabels.append(label)
            newHandles.append(handle)
    ax.legend(newHandles, newLabels)


def minFOVAngle(x_m, y_m, yaw_rad, poly):
    """
    Minimum facing angle of pose(x,y,theta) to a polygon
    """
    vector = np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
    min_angle = 10
    min_vertex = None
    for i, vertex in enumerate(poly):
        p2v = np.array([vertex[0] - x_m, vertex[1] - y_m])
        p2v = p2v / np.linalg.norm(p2v)
        angle = np.arccos(np.dot(vector, p2v))
        if angle < min_angle:
            min_angle = angle
            min_vertex = i
    if min_angle >= np.pi/2:
        return None
    pose = np.array([[x_m, y_m]])
    d = poly[min_vertex] - pose
    if np.linalg.norm(d) < 50:
        bound = d * 1.5   + pose
    else:
        bound = None
    return bound
