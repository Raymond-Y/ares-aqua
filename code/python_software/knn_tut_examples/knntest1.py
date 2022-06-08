# import numpy as np
# import matplotlib.pyplot as plt
# import pandas as pd
# from sklearn.neighbors import KNeighborsClassifier
# names = ['X_calculated', 'Y_calculated', "actual_values"]
# dataset = pd.read_csv('./out.csv', names=names)
# X = dataset.iloc[1:, 1:].values
# y = dataset.iloc[1:, 0].values
# print(X)
# print(y)
# print(type(X))
# print(type(y))
# classifier = KNeighborsClassifier(n_neighbors=1)
# classifier.fit(X, y)

# predict= (classifier.predict([[5.7,12]]))[0]
# print(predict)

# predict_split = predict.split("-")
# knn_x = float(predict_split[0])
# knn_y = float(predict_split[1])
# print(knn_x)
# print(knn_y)


self.names = ['X_calculated', 'Y_calculated', "actual_values"]
self.dataset = pd.read_csv('./TrainingData.csv', names=names)
self.trainingDataset = dataset.iloc[1:, 1:].values
self.classifierDataset = dataset.iloc[1:, 0].values
self.classifier = KNeighborsClassifier(n_neighbors=1)
self.classifier.fit(X, y)



predict= self.classifier.predict([[5.7,12]]))[0]
knn_x = float(predict_split[0])
knn_y = float(predict_split[1])
