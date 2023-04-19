#!/usr/bin/env python3

original = open("model.sdf",'r')
filedata = original.read()
original.close()

newdata = filedata.replace("0.02 0.02 0.01","1.0 1.0 1.0")

f = open("models/prova.sdf",'w+')
f.write(newdata)
f.close()