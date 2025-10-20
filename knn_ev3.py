# /home/robot/knn_ev3.py
import math, os

def _clip(x, a=0.0, b=1.0): 
    return a if x < a else (b if x > b else x)

class KNNPortable:
    """
    KNN que puede trabajar en dos modos:
      - Modo 'calib': usa blancos/negros medidos al inicio para normalizar a [0,1] y NO requiere mu/sigma.
      - Modo 'zscore': usa mu/sigma exportados (como antes).
    """
    def __init__(self, path=None, k=5, dist="euclidean", mode="calib"):
        self.k = int(k)
        self.dist = dist
        self.mode = mode  # "calib" o "zscore"

        # --- resolver carpeta del modelo ---
        base = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(base, "export_knn") if path is None else os.path.abspath(path)

        fx = os.path.join(model_dir, "knn_X.csv")
        fy = os.path.join(model_dir, "knn_y.csv")
        fcls = os.path.join(model_dir, "knn_classes.csv")

        for f in (fx, fy, fcls):
            if not os.path.exists(f):
                raise FileNotFoundError(f"Falta: {f}")

        self.X = self._read_matrix(fx)     # prototipos/ejemplos ya en el mismo espacio que predict()
        self.y = self._read_vector(fy)
        self.classes = self._read_classes(fcls)

        # Solo si usas modo zscore
        if self.mode == "zscore":
            fmu  = os.path.join(model_dir, "knn_mu.csv")
            fsig = os.path.join(model_dir, "knn_sigma.csv")
            for f in (fmu, fsig):
                if not os.path.exists(f):
                    raise FileNotFoundError(f"Falta: {f} (requerido para mode='zscore')")
            self.mu = self._read_vector(fmu)
            self.sigma = [ (s if abs(s) > 1e-9 else 1.0) for s in self._read_vector(fsig)]
            if len(self.mu) != len(self.X[0]):
                raise ValueError("Dimensión mu vs X no coincide")
        else:
            # modo calib: mu/sigma NO usados
            self.mu = None
            self.sigma = None

        # blancos/negros (se fijan desde tu programa principal tras calibrar)
        self.white = [1.0,1.0,1.0]  # placeholders
        self.black = [0.0,0.0,0.0]

    # ---- setters para pasar la calibración medida en tu setup ----
    def set_calibration(self, whiteL, whiteC, whiteR, blackL, blackC, blackR):
        self.white = [float(whiteL), float(whiteC), float(whiteR)]
        self.black = [float(blackL), float(blackC), float(blackR)]
        # evita división por cero
        for i in range(3):
            if abs(self.white[i]-self.black[i]) < 1e-6:
                self.white[i] = self.black[i] + 1.0

    # -------------- IO helpers --------------
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

    # -------------- distancias --------------
    def _dist(self, a, b):
        if self.dist == "cityblock":
            return sum(abs(ai-bi) for ai,bi in zip(a,b))
        elif self.dist == "chebychev":
            return max(abs(ai-bi) for ai,bi in zip(a,b))
        else:
            s = 0.0
            for ai,bi in zip(a,b):
                d = ai-bi
                s += d*d
            return math.sqrt(s)

    # -------------- normalizaciones --------------
    def _norm_calib(self, L, C, R):
        # a [0,1] usando blancos/negros medidos
        WL, WC, WR = self.white
        BL, BC, BR = self.black
        nL = _clip((L-BL)/(WL-BL))
        nC = _clip((C-BC)/(WC-BC))
        nR = _clip((R-BR)/(WR-BR))
        # features
        return [nL, nC, nR, nL-nC, nR-nC, nL-nR]

    def _norm_zscore(self, feat):
        # feat ya es [L,C,R,L-C,R-C,L-R] crudo
        return [(feat[i]-self.mu[i])/self.sigma[i] for i in range(len(self.mu))]

    # -------------- inferencia --------------
    def predict(self, L=None, C=None, R=None, feat=None):
        """
        Modo calib: pasa L,C,R (crudos) y NO 'feat'
        Modo zscore: pasa 'feat' = [L,C,R,L-C,R-C,L-R] crudo
        """
        if self.mode == "calib":
            assert L is not None and C is not None and R is not None, "En mode='calib' pasa L,C,R"
            x = self._norm_calib(L, C, R)
        else:
            assert feat is not None and len(feat)==6, "En mode='zscore' pasa feat de 6D"
            x = self._norm_zscore(feat)

        # KNN
        dists = []
        for i in range(len(self.X)):
            d = self._dist(x, self.X[i])
            dists.append((d, self.y[i]))
        dists.sort(key=lambda t: t[0])

        k = min(self.k, len(dists))
        votes = {}
        for i in range(k):
            lab = int(dists[i][1])
            votes[lab] = votes.get(lab, 0) + 1

        top = sorted(votes.items(), key=lambda t: (-t[1],))
        best_labels = [lab for lab,cnt in top if cnt==top[0][1]]
        if len(best_labels) == 1:
            lab = best_labels[0]
        else:
            sums = {lab:0.0 for lab in best_labels}
            for i in range(k):
                d, lab_i = dists[i]
                lab_i = int(lab_i)
                if lab_i in sums:
                    sums[lab_i] += d
            lab = min(sums.items(), key=lambda t: t[1])[0]
        return self.classes[lab-1]
