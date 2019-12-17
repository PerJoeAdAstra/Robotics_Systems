import csv
import matplotlib.pyplot as plt

no_correct = 0
no_incorrect = 0

total_offset = 0
average_offset = 0


def calculateAccuracy(reading, theta, actualAccuracy):
    if (abs(float(reading) - float(actualAccuracy)) < 30):
        return True
    else:
        return False

def calculateAccuracy2(reading, theta, actualAccuracy, total_offset):
    print(abs(float(reading) - float(actualAccuracy)), theta)
    total_offset += abs(float(reading) - float(actualAccuracy))
    return total_offset 


#  float accuracy = 0;
#   float objective_measurement = 0;
#   for (int i = 0; i < measurements.size(); i++){
#     objective_measurement = abs(270*sin(radsToDegs(measurements[i].theta)));
#     Serial.println((String)objective_measurement + ", " + (String)measurements[i].distance_measured );
#   }


x = []
y = []

with open('one-sensor-bad.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        y.append(float(row[0]) - 500)
        x.append(float(row[1]))

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