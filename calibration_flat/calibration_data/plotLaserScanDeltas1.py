import matplotlib.pyplot as plt
import csv


y = []


with open('delta1.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        y.append(float(row[0]))

x = range(len(y))
plt.scatter(x,y, label='Laser Scan Data')
plt.xlabel('x')
plt.ylabel('y')
plt.xlim([0, len(y)])
plt.ylim([0, 1])
plt.title('Deltas Reduced Ranges Laser Scan 1')
plt.legend()
plt.show()