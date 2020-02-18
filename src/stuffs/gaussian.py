from scipy.spatial import ConvexHull
from scipy.spatial import Delaunay

import numpy as np
import quadpy
import tripy


def minkowskiSum(obj1, obj2):
    """
        Minkowski sum of two polygon objects
        Args:
            obj1, obj2: (n,2) array of corner point
        Return:
            poly: (n,2) array of minkowski polygon vertices centered at (0, 0)
            bound: [min_x, min_y] max/min signed distances from vertices
                   [max_x, max_y] to center of polygon
    """
    assert obj1.ndim == 2 and obj1.shape[1] == 2
    assert obj2.ndim == 2 and obj2.shape[1] == 2

    poly = np.array([], dtype=np.float).reshape(0, 2)
    for p1 in obj1:
        for p2 in obj2:
            poly = np.vstack([poly, np.array([p1+p2])])
    poly00 = poly - np.mean(obj1, axis=0) - np.mean(obj2, axis=0)
    hull = ConvexHull(poly00)
    poly = poly00[hull.vertices]
    bound = {'max': hull.max_bound, 'min': hull.min_bound}

    assert poly.ndim == 2 and poly.shape[1] == 2

    return poly, bound


def pdfExplicit(point2D, mean, cov):
    """
        Explicit non-matrix multiplication
            for Probability density function (PDF) of 2D Gaussian
        Args:
            point2D: (2,) vector point
            mean: (2,) vector mean of the Gaussian distribution
            cov: (2,2) array covariance of the Gaussian distribution
        Return:
            prob: value of the PDF of given point
    """
    assert mean.shape == (2,)
    assert cov.shape == (2, 2)

    x = point2D[0] - mean[0]
    y = point2D[1] - mean[1]
    detC = cov[0, 0]*cov[1, 1] - cov[0, 1]*cov[1, 0]
    e = (cov[1, 1]*(x**2) + (-cov[0, 1]-cov[1, 0])*x*y
         + cov[0, 0]*(y**2)) / detC
    prob = 1 / (2 * np.pi * np.sqrt(abs(detC))) * np.exp(-0.5 * e)

    return prob


def pdf(point2D, mean, cov):
    """
        Probability density function (PDF) of 2D Gaussian
        Args:
            point2D: (2,) vector point
            mean: (2,) vector mean of the Gaussian distribution
            cov: (2,2) array covariance of the Gaussian distribution
        Return:
            prob: value of the PDF of given point
    """
    assert point2D.shape == (2,)
    assert mean.shape == (2,)
    assert cov.shape == (2, 2)

    e = np.matmul(np.matmul((point2D - mean),
                            np.linalg.inv(cov + np.diag([1e-9, 1e-9]))),
                  np.transpose(point2D - mean))
    prob = 1 / (2 * np.pi * np.sqrt(np.linalg.det(cov))) * np.exp(-0.5 * e)

    assert np.isscalar(prob)

    return prob


def polyIntegratePdf(poly, mean, cov, eps=1.0e-2, method=None):
    """
        Integrate a PDF with given mean and covariance over a convex polygon
        Args:
            poly: (n,2) array of corner points of convex polygon
            mean: (2,) vector mean of the Gaussian distribution
            cov: (2,2) array covariance of the Gaussian distribution
        Return:
            I: result of the integral
    """
    assert poly.ndim == 2 and poly.shape[1] == 2
    assert mean.shape == (2,)
    assert cov.shape == (2, 2)

    if method == 'simulation':
        count = 10000
        samples = np.random.multivariate_normal(mean, cov, count)
        prob = np.count_nonzero(in_hull(samples, poly)) / count

        assert np.isscalar(prob)

        return prob
    else:
        areaSum = 0.0
        triangles = np.asarray(tripy.earclip(poly))
        for triangle in triangles:
            sol, _ = quadpy.triangle.integrate_adaptive(
                lambda x: pdfExplicit(x, mean, cov),
                triangle,
                eps)
            areaSum += sol

        assert np.isscalar(areaSum)

        return min(areaSum, 1)


def in_hull(p, poly):
    """
    Test if points list p in poly
    """
    poly = Delaunay(poly)
    return poly.find_simplex(p) >= 0


def gaussian_testRectangle():
    """
    Test integral of bvn over polygon
    Test case:
        zero mean and unit covariance 2D Gaussian
        area with [xlim, ylim] is rectangle boundary from 0 to 1
    Method used:
        scipy.stats.mvnmvnun: closed form solution for rectangle bound
        scipy.integrate.dblquad: double integral
    """
    from scipy.stats import mvn
    import scipy.integrate as integrate
    import time
    # set values
    mean = np.array([0, 0])
    cov = np.eye(2)

    # mvnun
    start_mvnun = time.time()
    lower = np.array([0, 0])
    upper = np.array([1, 1])

    area_mvnun, _ = mvn.mvnun(lower, upper, mean, cov)
    time_mvnun = time.time() - start_mvnun

    # dblquad
    def pdfDBL(x, y, mean, cov):
        x1 = x - mean[0]
        x2 = y - mean[1]
        detC = cov[0, 0]*cov[1, 1]-cov[0, 1]*cov[1, 0]
        e = (cov[1, 1]*(x1**2) + (-cov[0, 1]-cov[1, 0])*x1*x2
             + cov[0, 0]*(x2**2)) / detC
        return 1 / (2 * np.pi * np.sqrt(abs(detC))) * np.exp(-0.5 * e)

    start_dblquad = time.time()
    area_dblquad = integrate.dblquad(
        lambda x, y: pdfDBL(x, y, mean, cov),
        lower[0], upper[0], lambda x: lower[1], lambda x: upper[1])[0]
    time_dblquad = time.time() - start_dblquad

    # polygon triangulation method
    start_poly = time.time()
    poly = np.array([[lower[0], lower[1]],
                     [upper[0], lower[1]],
                     [upper[0], upper[1]],
                     [lower[0], upper[1]]])
    area_poly = polyIntegratePdf(poly, mean, cov, eps=1.0e-5)
    time_poly = time.time() - start_poly

    # print output
    print("Test integral over rectangle bound :")
    print("Scipy mvnun: {:5.6f}".format(area_mvnun),
          " | time: {:6.9f} s".format(time_mvnun))
    print("Scipy dblquad: {:5.6f}".format(area_dblquad),
          " | time: {:6.9f} s".format(time_dblquad))
    print("Polygon integral: {:5.6f}".format(area_poly),
          " | time: {:6.9f} s".format(time_poly),
          " \nerror: {:5.8f}".format(abs(area_mvnun - area_poly)))


def gaussian_testPoly():
    """
        Test integral of bvn over polygon
        Test case:
            zero mean and unit covariance 2D Gaussian
            area with [xlim, ylim] is rectangle boundary from 0 to 1
        Method used:
            scipy.stats.mvnmvnun: closed form solution for rectangle bound
            scipy.integrate.dblquad: double integral
    """
    from scipy.stats import mvn
    import time
    # set values
    mean = np.array([0, 0])
    cov = np.eye(2)

    # mvnun
    start_mvnun = time.time()
    area_mvnun1, _ = mvn.mvnun(np.array([0, 0]), np.array([1, 1]), mean, cov)
    area_mvnun2, _ = mvn.mvnun(np.array([-1, -1]), np.array([0, 0]), mean, cov)
    area_mvnun3, _ = mvn.mvnun(np.array([0, -1]), np.array([1, 0]), mean, cov)
    area_mvnun = area_mvnun1 + area_mvnun2 + area_mvnun3
    time_mvnun = time.time() - start_mvnun

    # polygon triangulation method
    start_poly = time.time()
    poly = np.array([[-1, -1], [1, -1], [1, 1],
                     [0, 1], [0, 0], [-1, 0]])
    area_poly = polyIntegratePdf(poly, mean, cov, eps=1.0e-5)
    time_poly = time.time() - start_poly

    # print output
    print("Test integral over hexapolygon :")
    print("Scipy mvnun: {:5.6f}".format(area_mvnun),
          " | time: {:6.9f} s".format(time_mvnun))
    print("Polygon integral: {:5.6f}".format(area_poly),
          " | time: {:6.9f} s".format(time_poly),
          " \nerror: {:5.8f}".format(abs(area_mvnun - area_poly)))


# uncomment the following to test

if __name__ == "__main__":
    print("------------------")
    gaussian_testRectangle()
    print("------------------")
    gaussian_testPoly()
    print("------------------")
