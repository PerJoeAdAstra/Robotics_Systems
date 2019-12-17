import csv
import matplotlib.pyplot as plt
import sys


no_correct = 0
no_incorrect = 0

total_offset = 0
average_offset = 0
length = 0

x = []
y = []

def calculateAccuracy(reading, theta, actualAccuracy):
    if (abs(float(reading) - float(actualAccuracy)) < 30):
        return True
    else:
        return False

def addDeviation(reading, theta, actualAccuracy, total_offset):
    # print(abs(float(reading) - float(actualAccuracy)), theta)
    total_offset += abs(float(reading) - float(actualAccuracy))
    return total_offset 


fileName = sys.argv[1]

with open('{}.csv'.format(fileName), 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        #calculating deviation
        total_offset = addDeviation(row[0],row[1],row[2], total_offset)
        #calculating accuracy
        if(calculateAccuracy(row[0],row[1],row[2])):
            no_correct+=1
        #add to pyplot
        y.append(float(row[0]) - 500)
        x.append(float(row[1]))
        length+=1

print("accuracy: ", float(no_correct)/float(length))
print("deviation: ", float(total_offset-500)/float(length))


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