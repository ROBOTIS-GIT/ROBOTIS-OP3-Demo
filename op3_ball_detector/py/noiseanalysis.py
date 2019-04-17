import os, sys
from PIL import Image

#
#Author: Brady Testa
#
#This script is designed to analyze the difference between a set of pictures under a given rectangle.
#Place the identical pictures in a folder called 'noiseinput' and run the script


inputFolder = os.listdir("noiseinput/")
print ("File list: ")
print(inputFolder)

print("The program will select a rectangular bound by being given two points to form the opposite diagonal vertices of a rectangle with\n")
##
##ax = int(input(" 1st X: "))
##ay = int(input(" 1st Y: "))
##bx = int(input(" 2nd X: "))
##by = int(input(" 2nd Y: "))

ax = 700
ay = 425
bx = 770
by = 515

rangeX= bx-ax
rangeY= by-ay
fc =0
#matrix = [ [ [0]*len(inputFolder) ] * rangeY ] * rangeX
matrix = [[ [0 for a in range(len(inputFolder))] for b in range(rangeY)] for c in range(rangeX)]
results = [0]*(rangeX*rangeY)



##
##for a in range(0, rangeX):
##    for b in range(0, rangeY):
##        for c in range(0, len(inputFolder)):
##            matrix[a][b][c] = int(0)
##
            
for file in inputFolder:
    print("Analysis underway of: " + file)

    toAnalyze = Image.open("noiseinput\\" + file)
    picture = toAnalyze.load()
   
    for px in range(ax, bx):
        for py in range(ay, by): #for each pixel
            
            #print(str(px-ax) + " " + str(py-ay) + " " + str(fc))
            temp = matrix[px-ax][py-ay][fc]
            
            matrix[px-ax][py-ay][fc] = temp + picture[px, py][2]
            #if py == 450 and px == 750:
                #print(temp)
               # print(matrix[px-ax][py-ay][fc])
                #print(str( picture[px, py][2]))
               # print('\n')

    fc = fc+1


for px in range(ax, bx):
    
    for py in range(ay, by): #for each pixel
        for cnt in range(0, fc):
            results[(px-ax)*rangeY + (py-ay)] =results[(px-ax)*rangeY + (py-ay)] +  matrix[px-ax][py-ay][cnt]
            #print((results[(px-ax)*rangeY + (py-ay)]))
        #print("\n")
        results[(px-ax)*rangeY + (py-ay)] = results[(px-ax)*rangeY + (py-ay)] /fc

grandsum = 0
for b in range(rangeX*rangeY): #std calc
    sum = 0
 
    for i in range(len(inputFolder)):
        sum += (matrix[int(b/int(rangeY))][b%rangeY][i]-results[b]) * (matrix[int(b/int(rangeY))][b%rangeY][i]-results[b])
    grandsum += (sum/len(inputFolder))
    if b % 100 == 0:  
        print(sum/len(inputFolder))
print("average of stds")
print(grandsum/(rangeX*rangeY))
#print(results)




