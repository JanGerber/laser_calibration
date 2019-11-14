import matplotlib.pyplot as plt
import csv

x = []
y = []

with open('reducedranges2.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))

px = []
py = []


with open('calibration_points.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        px.append(float(row[0]))
        py.append(float(row[1]))

px = px[3:6]
py = py[3:6]


plt.scatter(x,y, label='Laser Scan Data')
plt.scatter(px,py, label='Extracted Points')
plt.xlabel('x')
plt.ylabel('y')
plt.xlim([0, 2.0])
plt.ylim([-0.75, 0.75])
plt.title('Reduced Ranges Laser Scan 2')
plt.legend()
plt.show()