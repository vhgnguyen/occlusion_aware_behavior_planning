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


class PlotScene(object):

    def __init__(self, core):
        self.core = core
        self._dAlpha = 0.1

    def plotScene(self):
        fig, ax = plt.subplots(figsize=(14, 6))
        pose = self.core.getCurrentEgoPos()

        self.plotRoad(ax=ax)
        self.plotFOV(ax=ax)
        self.plotStaticObject(ax=ax)
        self.plotPedestrianCross(ax=ax)
        self.plotEgoVehicle(ax=ax)
        self.plotPedestrian(ax=ax)
        self.plotVehicle(ax=ax)

        handleLegend(ax=plt)
        ax.axis('equal')
        ax.set_xlim(pose[0]-40, pose[0]+40)
        ax.set_ylim(pose[1]-20, pose[1]+20)
        ax.set_xlabel("x[m]")
        ax.set_ylabel("y[m]")
        plt.show()

    def plotPedestrian(self, ax):
        # plot other pedestrian
        pedesList = self.core.exportCurrentPedestrian()
        hypoPList = self.core.exportHypoPedestrian()

        for hypoP in hypoPList:
            hp = hypoP['p']
            a = 0.7
            b = len(hp)
            for i in range(0, b, int(b/5)):
                hpp = hp[i]
                a = max(a-self._dAlpha, 0.1)
                pose = hpp['pos']
                lw = hpp['std']
                plotEllipse(
                    x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                    facecolor='pink', edgecolor='tab:pink', alpha=a, ax=ax)

            hc = hypoP['c']
            plotPolygon(
                hc['poly'], facecolor='pink', edgecolor='k',
                alpha=1, label="hypothesis pedestrian", ax=ax,
                heading=True, hcolor='k')

        for pedes in pedesList:
            c = pedes['c']
            p = pedes['p']

            a = 0.7
            b = len(p)
            for i in range(0, b, int(b/5)):
                pp = p[i]
                a = max(a-self._dAlpha, 0.1)
                pose = pp['pos']
                lw = pp['std']
                if c['visible']:
                    plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='lime', edgecolor='g', alpha=a, ax=ax)
                else:
                    plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='salmon', edgecolor='r', alpha=a, ax=ax)

            if c['visible']:
                plotPolygon(
                    c['poly'], facecolor='pink', edgecolor='k',
                    alpha=1, label="detected pedestrian", ax=ax,
                    heading=True, hcolor='lime')
            else:
                plotPolygon(
                    c['poly'], facecolor='pink', edgecolor='k',
                    alpha=1, label="undetected pedestrian", ax=ax,
                    heading=True, hcolor='r')

    def plotVehicle(self, ax):
        # plot other vehicle
        vehicleList = self.core.exportCurrentVehicle()
        hypoList = self.core.exportHypoVehicle()

        for hypo in hypoList:
            hp = hypo['p']
            a = 0.7
            b = len(hp)
            for i in range(0, b, int(b/5)):
                hpp = hp[i]
                a = max(a-self._dAlpha, 0.1)
                pose = hpp['pos']
                lw = hpp['std']
                plotEllipse(
                    x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                    facecolor='yellow', edgecolor='gold', alpha=a, ax=ax)

            hc = hypo['c']
            plotPolygon(
                    hc['poly'], facecolor='gold', edgecolor='k',
                    alpha=1, label="hypothesis vehicle", ax=ax,
                    heading=True, hcolor='k')

        for veh in vehicleList:
            c = veh['c']
            p = veh['p']

            a = 0.7
            b = len(p)
            for i in range(0, b, int(b/5)):
                pp = p[i]
                a = max(a-self._dAlpha, 0.1)
                pose = pp['pos']
                lw = pp['std']
                if c['visible']:
                    plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='lime', edgecolor='g', alpha=a, ax=ax)
                else:
                    plotEllipse(
                        x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                        facecolor='lightcoral', edgecolor='r', alpha=a, ax=ax)

            if c['visible']:
                plotPolygon(
                    c['poly'], facecolor='gold', edgecolor='k',
                    alpha=1, label="detected vehicle", ax=ax,
                    heading=True, hcolor='lime')
            else:
                plotPolygon(
                    c['poly'], facecolor='gold', edgecolor='k',
                    alpha=1, label="undetected vehicle", ax=ax,
                    heading=True, hcolor='r')

    def plotRoad(self, ax):
        # plot road
        for road in self.core._env._l_road:
            # draw left boundary
            plotLine(
                road.left, ax=ax, color='k', linestyle='-')
            # draw right boundary
            plotLine(
                road.right, ax=ax, color='k', linestyle='-')
            # draw lane in dashed
            plotLine(
                road.lane, ax=ax, color='gray', linestyle='--')

    def plotFOV(self, ax):
        # plot FOV
        plotPolygon(
            self.core.getCurrentFOV(), facecolor='gainsboro', edgecolor='lightcoral',
            alpha=0.7, ax=ax, label="FOV")

    def plotStaticObject(self, ax):
        # plot static object
        objectList = self.core._env._l_staticObject
        for obj in objectList:
            plotPolygon(
                obj._poly, facecolor='gray', edgecolor='gray',
                alpha=1, label=None, ax=ax)

    def plotPedestrianCross(self, ax):
        # plot pedestrian cross
        crossList = self.core._env._l_cross
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
                cPoly = np.array([p1, p2, p3, p4])
                plotPolygon(
                    cPoly, facecolor='k', edgecolor='k',
                    alpha=0.5, label=None, ax=ax,
                    )

    def plotEgoVehicle(self, ax):
        # plot ego vehicle       
        p = self.core.getPredictEgo()
        b = len(p)
        a = 0.7
        for i in range(0, b, int(b/5)):
            pp = p[i]
            a = max(a-self._dAlpha, 0.1)
            pose = pp['pos']
            lw = pp['std']
            plotEllipse(
                x=pose[0], y=pose[1], a=pose[2], l=lw[0]*2, w=lw[1]*2,
                facecolor='cornflowerblue', edgecolor='b', alpha=a, ax=ax)
        plotPolygon(
            self.core.getCurrentEgoPoly(), facecolor='b', edgecolor='b',
            alpha=1, label='ego vehicle', ax=ax,
            heading=True, hcolor='k')
