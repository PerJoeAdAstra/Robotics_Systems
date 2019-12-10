import csv

no_correct = 0
no_incorrect = 0

def calculateAccuracy(reading, theta, actualAccuracy):
    if (abs(float(reading) - float(actualAccuracy)) < 30):
        return True
    else:
        return False

#  float accuracy = 0;
#   float objective_measurement = 0;
#   for (int i = 0; i < measurements.size(); i++){
#     objective_measurement = abs(270*sin(radsToDegs(measurements[i].theta)));
#     Serial.println((String)objective_measurement + ", " + (String)measurements[i].distance_measured );
#   }

with open('measurements4.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    for row in csv_reader:
        print(row)
        if(calculateAccuracy(row[0], row[1], row[2])):
            no_correct+=1
        else:
            no_incorrect+=1


print(no_correct)
print(no_incorrect)
print(no_correct/36.0)