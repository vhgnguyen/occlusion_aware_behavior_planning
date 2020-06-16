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

    gl.glLineWidth(0.05)
    if fill:
        gl.glPolygonMode(gl.GL_FRONT, gl.GL_FILL)
    else:
        gl.glPolygonMode(gl.GL_FRONT, gl.GL_LINE)

    if poly.shape[0] == 4:
        gl.glBegin(gl.GL_QUADS)
        for p in poly:
            gl.glVertex3f(p[0], p[1], 0)
        gl.glEnd()
    else:
        gl.glBegin(gl.GL_POLYGON)
        for p in poly:
            gl.glVertex3f(p[0], p[1], 0)
        gl.glEnd()
    gl.glPolygonMode(gl.GL_FRONT, gl.GL_FILL)

def drawHeading(poly, color, alpha, fill=True):
    if poly is None:
        return
    triangle = 0.5 * np.array([poly[0] + poly[1],
                               poly[1] + poly[2],
                               poly[2] + poly[3]])
    drawPoly(triangle, color=color, alpha=alpha, fill=fill)


def drawLine(line, color, alpha, lineWidth, strip=False):
    if line is None or line.shape[0] != 2:
        return
    setColor(color=color, alpha=alpha)
    gl.glLineWidth(lineWidth)
    if strip:
        gl.glLineWidth(0.1)
        gl.glLineStipple(1, 0xAAAA)
        gl.glEnable(gl.GL_LINE_STIPPLE)
        gl.glBegin(gl.GL_LINES)
        gl.glVertex2f(line[0][0], line[0][1])
        gl.glVertex2f(line[1][0], line[1][1])
        gl.glEnd()
        gl.glDisable(gl.GL_LINE_STIPPLE)
    else:
        gl.glLineWidth(0.1)
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


def plotEllipse(x, y, a, l, w,
                facecolor, edgecolor, alpha, label=None, ax=plt):
    e = Ellipse(xy=(x, y), width=l, height=w, angle=np.rad2deg(a), linewidth=0.5,
                edgecolor=edgecolor, facecolor=facecolor, alpha=alpha)
    ax.add_patch(e)


def plotLine(line, ax=plt, **kwargs):
    if line is None:
        return
    ax.plot(line[:, 0], line[:, 1], **kwargs)


def handleLegend(ax=plt):
    handles, labels = ax.gca().get_legend_handles_labels()
    newLabels, newHandles = [], []
    for handle, label in zip(handles, labels):
        if label not in newLabels:
            newLabels.append(label)
            newHandles.append(handle)
    ax.legend(newHandles, newLabels)
