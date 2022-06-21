from Array2D import  Array2D
from Point import Point
from AStar import AStar
import time  # 引入time模块

if __name__ == '__main__':
    #创建一个10*10的地图
    row = 100 #y
    col = 100 #x
    map2d=Array2D(row,col)
    #设置障碍
    for i in range(0,row-2):
    #for i in range(1, 4):
        map2d.data[i][3] =100
        map2d.data[i+2][col-3] = 100
        #map2d.data[row-1-i][col-3] = 100
    #显示地图当前样子

    map2d.showArray2D()
    #创建AStar对象,并设置起点为0,0终点为9,0
    startPoint = Point(0,0);
    goalPoint = Point(col-1,row-1);
    #goalPoint = Point(5, 2);
    # goalPoint = Point(5,2);
    aStar=AStar(map2d,startPoint,goalPoint)
    #开始寻路
    startTime = time.time()
    pathList=aStar.start()
    endTime = time.time()
    deltaTime = endTime - startTime
    print(f"delta time = {deltaTime}")
    #遍历路径点,在map2d上以'8'显示
    len = len(pathList)
    print("len= ",len)
    for point in pathList:
        map2d[point.y][point.x]=7
        #print(point)
    print("----------------------")
    #再次显示地图
    map2d.showArray2D()
    print(f"delta time = {deltaTime}")
