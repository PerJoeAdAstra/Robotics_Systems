import csv
import matplotlib.pyplot as plt
import sys


no_correct_15 = 0
no_correct_30 = 0

total_offset = 0
average_offset = 0
length = 0

x = []
y = []

def calculateAccuracy15(reading, theta, actualAccuracy):
    if (abs(float(reading) - float(actualAccuracy)) <= 15):
        return True
    else:
        return False

def calculateAccuracy30(reading, theta, actualAccuracy):
    if (abs(float(reading) - float(actualAccuracy)) <= 30):
        return True
    else:
        return False

def addDeviation(reading, theta, actualAccuracy, total_offset):
    # print(abs(float(reading) - float(actualAccuracy)), theta)
    total_offset += abs(float(reading) - float(actualAccuracy))
    return total_offset


fileName = sys.argv[1]

with open('new-readings/{}.csv'.format(fileName), 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        #calculating deviation
        total_offset = addDeviation(row[0],row[1],row[2], total_offset)
        #calculating accuracy
        if(calculateAccuracy15(row[0],row[1],row[2])):
            no_correct_15+=1
        if(calculateAccuracy30(row[0],row[1],row[2])):
            no_correct_30+=1
        #add to pyplot
        y.append(float(row[0]) - float(row[2]))
        x.append(float(row[1]))
        length+=1

print("Accuracy - 15mm window: ", float(no_correct_15)/float(length))
print("Accuracy - 30mm window: ", float(no_correct_30)/float(length))
print("Deviation: ", float(total_offset)/float(length))


plt.ylabel('Deviation (mm)')
plt.xlabel('Theta (radians)')
plt.plot(x, y)
plt.show()


# with open('measurements-1-sensor-circle.csv', 'r') as csv_file:
#     csv_reader = csv.reader(csv_file)
#     for row in csv_reader:
#         # total_offset = calculateAccuracy2(row[0], row[1], row[2], total_offset)
#         x.append()
#     average_offset = float(total_offset)/36
#     print(average_offset)