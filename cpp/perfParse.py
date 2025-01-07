import re
from matplotlib import pyplot as plt

f = open("log.txt", "r")

dataArr = []

for line in f.readlines():
    searchRes = re.search(r"(\d+)", line)
    if searchRes: dataArr.append(int(searchRes.group(1))); 

f.close()

# plt.hist(dataArr, 5)
plt.boxplot(dataArr, vert=False)
plt.show()