from knn_ev3 import KNNPortable

knn = KNNPortable("./export_knn", k=5, dist="euclidean", mode="calib")
knn.set_calibration(whiteL, whiteC, whiteR, blackL, blackC, blackR)
cls = knn.predict(L=L, C=C, R=R)   # devuelve "LEFT"/"CENTER"/"RIGHT"