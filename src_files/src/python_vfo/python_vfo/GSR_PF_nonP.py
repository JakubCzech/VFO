import numpy as np


class GSR:
    def __init__(self, sigma):
        self.sigma = sigma

    def calc_lambda_ellipse(self):
        A = 1.0
        B = 2.0
        F = lambda x, y: self.sigma * (
            ((x**2) / (A**2)) + ((y**2) / (B**2)) - 1.0
        )
        Fx = lambda x, y: self.sigma * 2 * x / (A**2)
        Fxx = lambda x, y: self.sigma * 2 / (A**2)
        Fy = lambda x, y: self.sigma * 2 * y / (B**2)
        Fyy = lambda x, y: self.sigma * 2 / (B**2)
        Fxy = lambda x, y: 0
        return F, Fx, Fxx, Fy, Fyy, Fxy

    def calc_lambda_superellipse(self):
        n = 10
        A = 0.7
        B = 0.6
        F = lambda x, y: self.sigma * (
            ((np.abs(x) ** n) / (A**n)) + ((np.abs(y) ** n) / (B**n)) - 1.0
        )
        Fx = lambda x, y: self.sigma * (
            (1 / (A**n)) * n * (x ** (n - 1)) * (np.sign(x) ** n)
        )
        Fxx = lambda x, y: self.sigma * (
            (1 / (A**n)) * n * (n - 1) * (x ** (n - 2)) * (np.sign(x) ** n)
        )
        Fy = lambda x, y: self.sigma * (
            (1 / (B**n)) * n * (y ** (n - 1)) * (np.sign(y) ** n)
        )
        Fyy = lambda x, y: self.sigma * (
            (1 / (B**n)) * n * (n - 1) * (y ** (n - 2)) * (np.sign(y) ** n)
        )
        Fxy = lambda x, y: 0
        return F, Fx, Fxx, Fy, Fyy, Fxy

    def calc_lambda_tangens(self):
        __y0 = 0.0
        __x0 = 1.0
        A = 5.0
        B = 1.0
        F = lambda x, y: self.sigma * ((y - __y0) - B * np.tanh(A * (x - __x0)))
        Fx = lambda x, y: -self.sigma * B * A * (1 - (np.tanh(A * (x - __x0))) ** 2)
        Fxx = (
            lambda x, y: self.sigma
            * 2
            * B
            * A
            * A
            * np.tanh(A * (x - __x0))
            * (1 - (np.tanh(A * (x - __x0))) ** 2)
        )
        Fy = lambda x, y: self.sigma
        Fyy = lambda x, y: 0
        Fxy = lambda x, y: 0
        return F, Fx, Fxx, Fy, Fyy, Fxy

    def test(self):
        print("GSR test")
        F = lambda x, y: self.sigma * (x**2 + ((y / 2) ** 2) - 1)
        Fx = lambda x, y: self.sigma * 2 * x
        Fxx = lambda x, y: self.sigma * 2
        Fy = lambda x, y: self.sigma * y
        Fyy = lambda x, y: self.sigma
        Fxy = lambda x, y: 0
        return F, Fx, Fxx, Fy, Fyy, Fxy
