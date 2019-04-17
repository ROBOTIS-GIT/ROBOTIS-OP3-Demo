import os, sys
from PIL import Image

#
#Author: Brady Testa
#
#Anaylzes the color of the picture w/ respect to the light of the picture. put the pictures in folder 'lightinput'
#and store the light values in light.csv
#

inputFolder = os.listdir("lightinput/")
print ("File list: ")
print(inputFolder)

ax = 650
ay = 370
bx = 690
by = 400
rangeX= bx-ax
rangeY= by-ay


fc =0

lightValues = [0] * len(inputFolder)
matrix = [[ [0 for a in range(len(inputFolder))] for b in range(rangeY)] for c in range(rangeX)]
results = [0]*(rangeX*rangeY)

for file in inputFolder:
    print("Analysis underway of: " + file)

    toAnalyze = Image.open("lightinput\\" + file)
    picture = toAnalyze.load()
   
    for px in range(ax, bx):
        for py in range(ay, by): #for each pixel

            temp = matrix[px-ax][py-ay][fc]
            
            matrix[px-ax][py-ay][fc] = temp + picture[px, py][2]


    fc = fc+1
for px in range(ax, bx):
    for py in range(ay, by): #for each pixel
        for i in range(len(inputFolder)):
             print(matrix[px-ax][py-ay][i])
        print('\n')
