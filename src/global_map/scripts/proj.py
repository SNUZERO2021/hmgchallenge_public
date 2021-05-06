#!/usr/bin/env python

import csv

def projection(lati, longi):
    lati = (lati-37.58) * 10000
    longi = (longi - 126.89) * 10000
    a = -1.0526565e-01*lati +  8.8334789e+00*longi +  3.4405369e-01
    b = 1.1098592e+01*lati +  8.3344474e-02*longi +  1.4646138e-01
    c = 4.8751599e-07*lati +  4.2407201e-08*longi +  1.0000000e+00
    return a/c, b/c

name_dict = {}
with open("../raw_data/names.csv", newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    line = 0
    for row in reader:
        line += 1
        #if line == 1:
        #    continue
        if len(row) != 2:
            print('something wrong, line {}, size {}'.format(line, len(row)))
            assert False
        if row[1] == "":
            continue
        name_dict[row[1]] = row[0]

curname = ""
curid = ""
points = []

with open("../raw_data/Coordinates.csv", newline='') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    line = 0
    for row in reader:
        line += 1
        if line == 1:
            continue
        if len(row) != 19:
            print('something wrong, line {}, size {}'.format(line, len(row)))
            assert False
        
        _id = row[1]
        if _id == "":
            continue
        if curid == _id:
            lati = float(row[18])
            longi = float(row[17])
            x, y = projection(lati, longi)
            points.append((x,y))
        else:
            if curid != "":
                with open("../segment/{}.txt".format(curname), 'w') as f:
                    for point in points:
                        f.write("{} {}\n".format(point[0], point[1]))
                    curid = ""
                    points = []

            if _id not in name_dict:
                continue

            curname = name_dict[_id]
            curid = _id
            points = []

            lati = float(row[18])
            longi = float(row[17])
            x, y = projection(lati, longi)
            points.append((x,y))

if curid != "":
    with open("../segment/{}.txt".format(curname), 'w') as f:
        for point in points:
            f.write("{} {}\n".format(point[0], point[1]))
        curid = ""
        points = []