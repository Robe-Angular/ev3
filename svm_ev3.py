# /home/robot/svm_ev3.py  (Python 3.5 compatible)
import os

def _clip(x, a=0.0, b=1.0):
    if x < a: return a
    if x > b: return b
    return x

class SVMPortable:
    def __init__(self, path=None, mode="calib"):
        """
        mode='calib' -> usa blancos/negros medidos para [0,1] y arma features de 6D
        (mismo pipeline que en el trainer).
        """
        base = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(base, "export_svm") if path is None else os.path.abspath(path)

        fW   = os.path.join(model_dir, "svm_W.csv")
        fb   = os.path.join(model_dir, "svm_b.csv")
        fcls = os.path.join(model_dir, "svm_classes.csv")

        for f in (fW, fb, fcls):
            if not os.path.exists(f):
                raise IOError("Falta: {}".format(f))

        self.W = self._read_matrix(fW)     # C x D
        self.b = self._read_vector(fb)     # C
        self.classes = self._read_classes(fcls)
        self.mode = mode
        self.white = [1.0,1.0,1.0]
        self.black = [0.0,0.0,0.0]

        # sanity
        if len(self.W)==0:
            raise ValueError("svm_W.csv vacío")
        D = len(self.W[0])
        if len(self.b) != len(self.W):
            raise ValueError("Dimension b no coincide con W")
        self.D = D

    def set_calibration(self, whiteL, whiteC, whiteR, blackL, blackC, blackR):
        self.white = [float(whiteL), float(whiteC), float(whiteR)]
        self.black = [float(blackL), float(blackC), float(blackR)]
        for i in range(3):
            if abs(self.white[i]-self.black[i]) < 1e-6:
                self.white[i] = self.black[i] + 1.0

    def _read_matrix(self, fname):
        M = []
        with open(fname,"r") as f:
            for line in f:
                line=line.strip()
                if not line: continue
                M.append([float(x) for x in line.split(",")])
        return M

    def _read_vector(self, fname):
        v = []
        with open(fname,"r") as f:
            for line in f:
                line=line.strip()
                if not line: continue
                parts = [p for p in line.split(",") if p!=""]
                v.extend([float(x) for x in parts])
        return v

    def _read_classes(self, fname):
        classes = []
        with open(fname,"r") as f:
            for line in f:
                line=line.strip()
                if not line: continue
                classes.append(line)
        return classes

    def _features_from_raw(self, L, C, R):
        WL, WC, WR = self.white
        BL, BC, BR = self.black
        nL = _clip((L-BL)/(WL-BL))
        nC = _clip((C-BC)/(WC-BC))
        nR = _clip((R-BR)/(WR-BR))
        return [nL, nC, nR, nL-nC, nR-nC, nL-nR]

    def _score(self, x, w, b):
        s = 0.0
        for i in range(len(x)):
            s += x[i] * w[i]
        return s + b

    def predict(self, L=None, C=None, R=None):
        if self.mode != "calib":
            raise ValueError("Slo mode='calib' soportado en EV3 para SVM.")
        if L is None or C is None or R is None:
            raise ValueError("Pasa L,C,R crudos")
        x = self._features_from_raw(L, C, R)  # 6D

        # scores OVR
        best_i = 0
        best_s = None
        for ci in range(len(self.W)):
            s = self._score(x, self.W[ci], self.b[ci])
            if (best_s is None) or (s > best_s):
                best_s = s
                best_i = ci
        return self.classes[best_i]
