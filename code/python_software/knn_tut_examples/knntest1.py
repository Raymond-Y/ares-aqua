import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.neighbors import KNeighborsClassifier
names = ['X_calculated', 'Y_calculated', "X_actual", "Y_actual"]
dataset = pd.read_csv('./knndata.csv', names=names)
X = dataset.iloc[:, :-2].values
y = dataset.iloc[:, 2].values
print(X)
print(y)
print(type(X))
print(type(y))
classifier = KNeighborsClassifier(n_neighbors=1)
classifier.fit(X, y)
#neigh = KNeighborsClassifier(n_neighbors=1)
#neigh.fit(X, y)
predict= (classifier.predict([[13.3,2.4]]))[0]
print(predict)

predict_split = predict.split("-")
knn_x = float(predict_split[0])
knn_y = float(predict_split[1])
print(knn_x)
print(knn_y)
