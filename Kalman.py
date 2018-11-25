class Kalman:

    # avarage error
    varVolt = 6
    # speed
    varProcess = 3
    Pc = 0.0
    G = 0.0
    P = 1.0
    Xp = 0.0
    Zp = 0.0
    Xe = 0

    def kalman(self, val):
        self.Pc = self.P + self.varProcess
        self.G = self.Pc / (self.Pc + self.varVolt)
        self.P = (1 - self.G) * self.Pc
        self.Xp = self.Xe
        self.Zp = self.Xp
        self.Xe = self.G * (val - self.Zp) + self.Xp
        return int(self.Xe)
