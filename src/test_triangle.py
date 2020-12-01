#%%
import numpy as np
from stuffs.gaussian import *
from scipy import special
import math
from scipy.integrate import quad, dblquad, fixed_quad, romberg
import quadpy
# %%
def pdfDBL(x, y, mean, cov):
       x1 = x - mean[0]
       x2 = y - mean[1]
       detC = cov[0, 0]*cov[1, 1]-cov[0, 1]*cov[1, 0]
       e = (cov[1, 1]*(x1**2) + (-cov[0, 1]-cov[1, 0])*x1*x2 + cov[0, 0]*(x2**2)) / detC
       return 1 / (2 * np.pi * np.sqrt(abs(detC))) * np.exp(-0.5 * e)

area_dblquad = dblquad(lambda x, y: pdfDBL(x, y, mean, cov),0, 1, lambda x: 0, lambda x: 1)[0]

# %%
def pdf(x, y):
       return 1 / (2*np.pi) * np.exp(-0.5*(x**2 + y**2))

res = dblquad(lambda x, y: pdf(x, y), 0, 1, lambda x: 0, lambda x: 1- x)[0]
print(res)
triangle = np.array([[0, 0], [1, 0], [0, 1]])
mean = np.array([0, 0])
cov = np.eye(2)
sol, _ = quadpy.triangle.integrate_adaptive(
       lambda x: pdfExplicit(x, mean, cov),
       triangle,
       eps=1e-8)
print(sol)
# %%
def pdf1D(x):
       return 0.5 / np.sqrt(2*np.pi) * np.exp(-0.5*(x**2)) * math.erf((1-x)/np.sqrt(2))

quad(pdf1D, 0, 1)

# %%
refTriangle = np.array([[0, 0], [1, 0], [0, 1]])
triangle = np.array([[0, 0], [-1, -4], [1, -2]])
#%%
# triangle method
mean = np.array([2, 2])
cov = np.array([[2, 0], [0, 2]])
sol, _ = quadpy.triangle.integrate_adaptive(
       lambda x: pdfExplicit(x, mean, cov),
       triangle,
       eps=1e-8)
print(sol)

# normalize mean variance method
from scipy.linalg import fractional_matrix_power
triangle1 = np.dot(fractional_matrix_power(cov, -0.5), (triangle[:]-mean).T).T
def pdfNormal(x):
       return 1 / (2 * np.pi) * np.exp(-0.5 * (x[0]**2 + x[1]**2))

sol, _ = quadpy.triangle.integrate_adaptive(lambda x: pdfNormal(x), triangle1, eps=1e-2)
print(sol)

#%%
# test 1D projection to reference area
def testRef(x, triangle):
       [a1, a2] = triangle[1]
       [b1, b2] = triangle[2]
       a = b1**2 + b2**2
       b = 2*a1*b1 + 2*a2*b2
       c = a1**2 + a2**2
       return 1/(2*np.pi)*det * np.exp(-0.5*(a*(x[0]**2) + b*x[0]*x[1] + c*(x[1]**2)))

sol, _ = quadpy.triangle.integrate_adaptive(lambda x: testRef(x, triangle1), refTriangle, eps=1e-8)
print(sol)

#%%
# 1D projection of triangle method
[a1, a2] = triangle1[1]
[b1, b2] = triangle1[2]
a = b1**2 + b2**2
b = 2*a1*b1 + 2*a2*b2
c = a1**2 + a2**2
det = abs(a1*b2- a2*b1)

def refPoly(x,a,b,c,det):
       e = (b*x)**2/(8*a) - c*x**2/2
       f = np.sqrt(a/2) * (1 - x + b*x/(2*a))
       g = np.sqrt(a/2) * b * x / (2*a)
       res = 1 / (2*np.pi) * np.sqrt(np.pi/(2*a)) * det * np.exp(e)*(math.erf(f) - math.erf(g))
       return res

quad(lambda x: refPoly(x,a,b,c,det), 0, 1)
# %%
poly = np.array([[-1, -1], [1, -1], [1, 1],
                     [0, 1], [0, 0], [-1, 0]])
triangles = np.asarray(tripy.earclip(poly))
#%%
triangles = np.array(
    [[[0.0, 0.0], [1.0, 0.0]], [[1.0, 0.0], [1.0, 1.0]], [[0.0, 1.0], [0.0, 1.0]]])
#%%
def pdfNormal(x):
       return 1 / (2 * np.pi) * np.exp(-0.5 * (x[0]**2 + x[1]**2))

sol, _ = quadpy.triangle.integrate_adaptive(lambda x: pdfNormal(x), triangles, eps=1e-3)

# %%
