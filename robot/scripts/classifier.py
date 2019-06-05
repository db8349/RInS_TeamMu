#!/usr/bin/env/ python3

def classify(url, digit1, digit2):
    from sklearn import svm
    import urllib.request
    file = urllib.request.urlopen(url).read().decode()
    lines = file.split("\n")

    atr1 = []
    atr2 = []
    classes = []
    data = []

    for line in lines[1:len(lines)-1]:
        splitLine = line.split(",")
        atr1.append(float(splitLine[0]))
        atr2.append(float(splitLine[1][1:]))
        classes.append(int(splitLine[2][1]))
        data.append([float(splitLine[0]), float(splitLine[1][1:])])

    classifier = svm.SVC(gamma = "scale")
    classifier.fit(data, classes)
    prediction = classifier.predict([[digit1, digit2]])
    if prediction == [0]: return "red"
    if prediction == [1]: return "green"
    if prediction == [2]: return "blue"
    return "yellow"

#print(classify("http://box.vicos.si/rins/h.txt", 5, 9))