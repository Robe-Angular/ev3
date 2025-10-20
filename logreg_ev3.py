# /home/robot/logreg_ev3.py
# Multiclass Logistic Regression (softmax) runtime, Python 3.5 compatible.
import os, math

def _clip(x, a=0.0, b=1.0):
    if x < a: return a
    if x > b: return b
    return x

class LogisticPortable:
    def __init__(self, path=None, mode="calib"):
        """
        mode='calib' -> use runtime white/black to normalize to [0,1],
                        then build 6D features [nL,nC,nR,nL-nC,nR-nC,nL-nR].
        """
        base = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(base, "export_logreg") if path is None else os.path.abspath(path)

        fW   = os.path.join(model_dir, "log_W.csv")
        fb   = os.path.join(model_dir, "log_b.csv")
        fcls = os.path.join(model_dir, "log_classes.csv")

        for f in (fW, fb, fcls):
            if not os.path.exists(f):
                raise IOError("Missing: {}".format(f))

        self.W = self._read_matrix(fW)    # C x D
        self.b = self._read_vector(fb)    # C
        self.classes = self._read_classes(fcls)
        self.mode = mode
        self.white = [1.0,1.0,1.0]
        self.black = [0.0,0.0,0.0]

        if len(self.W)==0:
            raise ValueError("log_W.csv is empty")
        if len(self.b) != len(self.W):
            raise ValueError("b size does not match W rows")
        self.C = len(self.W)
        self.D = len(self.W[0])

    def set_calibration(self, whiteL, whiteC, whiteR, blackL, blackC, blackR):
        self.white = [float(whiteL), float(whiteC), float(whiteR)]
        self.black = [float(blackL), float(blackC), float(blackR)]
        for i in range(3):
            if abs(self.white[i]-self.black[i]) < 1e-6:
                self.white[i] = self.black[i] + 1.0

    # ---------- IO ----------
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

    # ---------- features ----------
    def _features_from_raw(self, L, C, R):
        WL, WC, WR = self.white
        BL, BC, BR = self.black
        nL = _clip((L-BL)/(WL-BL))
        nC = _clip((C-BC)/(WC-BC))
        nR = _clip((R-BR)/(WR-BR))
        return [nL, nC, nR, nL-nC, nR-nC, nL-nR]

    # ---------- math ----------
    def _score_row(self, w, b, x):
        s = 0.0
        for i in range(len(x)):
            s += w[i] * x[i]
        return s + b

    def _softmax(self, scores):
        # numerical stability: subtract max
        m = max(scores)
        exps = []
        ssum = 0.0
        for s in scores:
            e = math.exp(s - m)
            exps.append(e)
            ssum += e
        probs = []
        if ssum <= 0.0: ssum = 1.0
        for e in exps:
            probs.append(e/ssum)
        return probs

    # ---------- inference ----------
    def predict(self, L=None, C=None, R=None):
        if L is None or C is None or R is None:
            raise ValueError("Pass raw L, C, R")
        x = self._features_from_raw(L, C, R)
        scores = []
        for ci in range(self.C):
            scores.append(self._score_row(self.W[ci], self.b[ci], x))
        probs = self._softmax(scores)
        best_i = 0
        best_p = probs[0]
        for i in range(1, len(probs)):
            if probs[i] > best_p:
                best_p = probs[i]
                best_i = i
        return self.classes[best_i]

    def predict_with_probs(self, L=None, C=None, R=None):
        if L is None or C is None or R is None:
            raise ValueError("Pass raw L, C, R")
        x = self._features_from_raw(L, C, R)
        scores = []
        for ci in range(self.C):
            scores.append(self._score_row(self.W[ci], self.b[ci], x))
        probs = self._softmax(scores)
        best_i = 0
        best_p = probs[0]
        for i in range(1, len(probs)):
            if probs[i] > best_p:
                best_p = probs[i]
                best_i = i
        return self.classes[best_i], probs, x
