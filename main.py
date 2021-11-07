# 本代码所有可视化部分，建议仅调试模块时候选择性使用，因为过多的图像绘制会对线条检测、色块检测、数字识别产生较大影响。
# 实测部署时建议关闭所有可视化部分。

# 2021-11-07
# Designed By CIDP-SX-VisionTeam


import sensor, image, time, math # 基础库
from pyb import UART,LED # 串口通信
from image import SEARCH_EX, SEARCH_DS # 模板匹配

sensor.reset()
sensor.set_framesize(sensor.QQVGA) # 适用于 OpenMV4 H7 处理能力的图像分辨率
sensor.set_pixformat(sensor.RGB565) # 获取彩色图 RGB565, 灰度图 GRAYSCALE
sensor.skip_frames(time = 2000) # 让初始化生效
clock = time.clock()

uart = UART(3,115200)   #定义串口3变量
uart.init(115200, bits=8, parity=None, stop=1) # 初始化串口信息

red_threshold = (18, 69, 18, 127, -5, 55) # 检测红色线条的LAB像素阈值(建议根据实际环境情况调整)

dectectionFlag = True # 是否进行过数字指令接受。(True为没有(初始状态)，False为已接收)
targetNum = None # 目标病房号数字初始化

lastValue = 80 # 默认道路分离线初始化值为画面中心
posResult = 0 # 默认没有检测到地面数字标签
driverProcess = [] # 用于存储小车转向信息(用于倒车)


# 绘图函数(用于调试可视化)【实测部署时建议关闭所有可视化部分】
def drowWindows(img, rois=None, lines=None):
    numRoi = [2,10,155,60] # 数字识别区域【记得和templateMatch函数内numRoi同步更改】
    #img.draw_rectangle([30,10,100,100])
    img.draw_rectangle(numRoi)
    # 绘制检测道路口形状的四个区域
    img.draw_rectangle([65, 3, 30, 10],color=(0,255,0)) # 上侧
    img.draw_rectangle([18, 15, 44, 24],color=(0,255,0)) # 左侧
    img.draw_rectangle([98, 15, 47, 24],color=(0,255,0)) # 右侧
    img.draw_rectangle([65, 49, 30, 60],color=(0,255,0)) # 下侧


# 模板匹配数字(1~8号)【return：Tuple】
def templateMatch(img, dectectionFlag, imgPath): # (图像、检测标志、模板路径)
    numRoi = [2,10,155,60] # 数字识别区域【记得和drowWindows函数内numRoi同步更改】
    img_bat = img.copy()
    img_bat.to_grayscale() # 彩色图转换为灰度图
    if dectectionFlag: # model_template #,roi=[5,10,150,50]
        template = image.Image("/target_template/"+str(imgPath)+".pgm") # 药房任务接受模板匹配
        r = img_bat.find_template(template, 0.80, roi=[30,10,100,100], step=1, search=SEARCH_EX)
        img.draw_rectangle([30,10,100,100])
    else:
        # 尝试匹配左模板
        template = image.Image("/model_template/"+str(imgPath)+"_l.pgm") # 病房任务模板匹配（左侧）
        r = img_bat.find_template(template, 0.80, roi=numRoi, step=1, search=SEARCH_EX) # roi=[5,10,150,50]
        if r == None: # 尝试匹配右模板
            template = image.Image("/model_template/"+str(imgPath)+"_r.pgm") # 病房任务模板匹配（右侧）
            r = img_bat.find_template(template, 0.80, roi=numRoi, step=1, search=SEARCH_EX)
    #img.draw_rectangle(numRoi)
    if r:
        img.draw_rectangle(r, color=(255,0,255)) # 图像中画出匹配框
    return r # 输出匹配结果(无结果为None,有结果为目标的(x,y,w,h))


## 检测红线路口【已弃用】【霍夫曼直线检测方案(因为检出不稳定连续，泛化能力弱，已弃用)】(返回0：未检测到路口 返回1：检测到十字路口 返回2：检测到T字路口)
#def detectionCrossRoad(img, red_threshold):
    #img.binary([red_threshold]) # 二值化图像，分离线条
    #img1.lens_corr(1.1) # 相机畸变矫正

    ## 像素点阈值，直线搜索时的进步尺寸的单位半径，直线搜索时的进步尺寸的单位角度
    #for l in img.find_lines(threshold = 12000, theta_margin = 55, rho_margin = 55): # 20000  55 55
        #print(l.x1(),l.y1(),l.theta(),l.magnitude(),l.rho())

        #if abs(l.theta()-90) < 30: # 横线检测(红色) （60~120）
            #print("检测横线角度为：",l.theta())
            #img.draw_line(l.line(), color = (255, 0, 0))
        #if l.theta() > 160 or l.theta() < 20: # 竖线检测(绿色) （160~180，0~20）
            #print("检测竖线角度为：",l.theta())
            #img.draw_line(l.line(), color = (0, 255, 0))
        ##img.draw_line(l.line(), color = (0, 255, 0))
    #img.draw_rectangle([5,10,150,50]) # 画预判框
    #img.draw_rectangle([55,10,50,50]) # 画预判框(160)
    #return 0


# 道路分界检测【return: Int 获取道路中间分界线X值】
def crossRoadSeparate(img, red_threshold, lastValue): # 引入了lastValue, 保证分界线的连续检出。
    img_bin = img.copy()
    img_bin.binary([red_threshold]) # 二值化图像，分离线条
    img_bin.lens_corr(1.1) # 相机畸变矫正【一般可以不用】
    min_degree = 20 # 霍夫变换角度最小值
    max_degree = 160 # 179 # 霍夫变换直线角度最大值
    pixel_threshold = 10000 # 像素点阈值
    theta_margins = 25 # 直线搜索时的进步尺寸的单位半径
    rho_margins = 25 # 直线搜索时的进步尺寸的单位角度
    for l in img_bin.find_lines(threshold = pixel_threshold, theta_margin = theta_margins, rho_margin = rho_margins):
        #print(l.x1(),l.y1(),l.theta(),l.magnitude(),l.rho()) # 调参看值用
        if l.theta() > max_degree or l.theta() < min_degree: # 竖线检测(绿色) （角度范围为 160~180，0~20）
            #img.draw_line(l.line(), color = (0, 255, 0)) # 显示中间线
            if l != None: # 若检测到直线，则视为道路分界线【这里有个小Trick,保证调试设置好合理阈值后，视为第一个检测到的直线作为分界线】
                return l.x1()
    return lastValue # 若本轮未检测到直线，则用最近历史值作为分界线


# 道路情况逻辑判断【return: Int 路口情况分类】
def trafficLogic(roiRegion):
    trafficResult = 0 # 初始化
    if roiRegion == [1,0,0,1]:
        #print("当前直线向前")
        trafficResult = 1
    elif roiRegion == [1,1,1,1]:
        #print("当前为十字路口")
        trafficResult = 2
    elif roiRegion == [0,1,1,1]:
        #print("当前正对T字路口")
        trafficResult = 3
    elif roiRegion == [1,1,0,1]:
        #print("当前T字路口向左")
        trafficResult = 4
    elif roiRegion == [1,0,1,1]:
        #print("当前T字路口向右")
        trafficResult = 5
    elif roiRegion == [0,0,0,1]:
        #print("已经到达道路尽头")
        trafficResult = 6
    elif roiRegion == [0,0,0,0]:
        #print("当前已经跑偏或未检测到赛道")
        trafficResult = 7
    return trafficResult # 返回逻辑判断结果


# 小车巡线【return: Float 小车偏离直线角度】
def lineFlowing(img):
    RED_THRESHOLD = [(18, 69, 18, 127, -5, 55)]
    # 每个roi为(x, y, w, h)，线检测算法将尝试找到每个roi中最大的blob的质心。
    # 然后用不同的权重对质心的x位置求平均值，其中最大的权重分配给靠近图像底部的roi，
    # 较小的权重分配给下一个roi，以此类推。

    #roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
    #weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
    #三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
    #如上图的最下方的矩形，即(0, 100, 160, 20, 0.7)
    ROIS = [ # [ROI, weight](在值的设置有意避免了边角处畸变引起的计算问题)
            (30, 90, 85, 20, 0.7), # 下侧检测区
            (0, 050, 160, 20, 0.3), # 中侧检测区
            (40, 005, 80, 10, 0.1) # 上侧检测区
           ]
    weight_sum = 0 #权值和初始化
    for r in ROIS: weight_sum += r[4]
    #计算权值和。遍历上面的三个矩形，r[4]即每个矩形的权值。
    most_pixels = 80 # 默认最小色块像素总量
    centroid_sum = 0
    #利用颜色识别分别寻找三个矩形区域内的线段
    for r in ROIS:
        blobs = img.find_blobs(RED_THRESHOLD, roi=r[0:4], merge=True)
        # r[0:4] is roi tuple.
        #找到视野中的线,merge=true,将找到的图像区域合并成一个
        #目标区域找到直线
        if blobs:
            # 查找像素最多的blob的索引。
            largest_blob = 0
            for i in range(len(blobs)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                if blobs[i].pixels() > most_pixels:
                    if blobs[i].rect()[2]/blobs[i].rect()[3] > 1: # 过滤横向的色块(通常它们是路口或横行的直线)
                        continue
                    most_pixels = blobs[i].pixels()
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于
                    #most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                    largest_blob = i
            # 在色块周围画一个矩形。
            img.draw_rectangle(blobs[largest_blob].rect(),color=(255,255,0))
            # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
            img.draw_cross(blobs[largest_blob].cx(),
                           blobs[largest_blob].cy(),
                           color=(255,255,0))
            centroid_sum += blobs[largest_blob].cx() * r[4] # r[4] is the roi weight.
            #计算centroid_sum，centroid_sum等于每个区域的最大颜色块的中心点的x坐标值乘本区域的权值
    center_pos = (centroid_sum / weight_sum) # Determine center of line.
    #中间公式
    # 将center_pos转换为一个偏角。我们用的是非线性运算，所以越偏离直线，响应越强。
    # 非线性操作很适合用于这样的算法的输出，以引起响应“触发器”。
    deflection_angle = 0 # 初始化机器人应该转的角度
    # 80是X的一半，60是Y的一半。
    # 下面的等式只是计算三角形的角度，其中三角形的另一边是中心位置与中心的偏差，相邻边是Y的一半。
    # 这样会将角度输出限制在-45至45度左右。（不完全是-45至45度）。
    deflection_angle = -math.atan((center_pos-80)/60)    #角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120.    #角度计算.80 60 分别为图像宽和高的一半，图像大小为QQVGA 160x120. #注意计算得到的是弧度值
    deflection_angle = math.degrees(deflection_angle) #将计算结果的弧度值转化为角度值
    # 现在你有一个角度来告诉你该如何转动机器人。通过该角度可以合并最靠近机器人的部分直线和远离机器人的部分直线，以实现更好的预测。
    print("解算偏离角度: %f" % deflection_angle)
    return deflection_angle


# 小车转向判断【return: Int 返回转向结果(0:未检测到 1:左转 2:右转)】
def turnLogic(targetNum, divideLine):
    turnResult = 0 # 初始化检测标志
    if (targetNum[0] - divideLine) > 0: # 若大于0，则目标在分界线右侧。反之则目标在分界线左侧
        turnResult = 2
    else:
        turnResult = 1
    return turnResult


# 检测红线路口【return: Int 路口情况分类】
def crossRoadDetection(img, RED_THRESHOLD):
    RED_THRESHOLD = [(18, 69, 18, 127, -5, 55)] #如果是黑线[(0, 64)] #如果是白线[(128，255)]
    ROIS = [
            (65, 3, 30, 10), # 上侧检测区
            (18, 15, 44, 24), # 左侧检测区
            (98, 15, 47, 24), # 右侧检测区
            (65, 49, 30, 60) # 下侧检测区
           ]
    most_pixels = 80 # 目标区块最小面积
    #利用颜色识别分别寻找三个矩形区域内的线段
    roiRegion = [] # 区域(用于存储转向逻辑判断)
    counterRoi = 0 # 默认从第一个区域开始计数(用于判断哪个区域存在道路线)
    for r in ROIS:
        redLineFlag = 0 # 红线存在标志位初始化
        counterRoi += 1
        blobs = img.find_blobs(RED_THRESHOLD, roi=r[0:4], merge=True) #找到视野中的线,merge=true,将找到的图像区域合并成一个
        #目标区域找到直线
        if blobs:
            # 查找像素最多的blob的索引。
            largest_blob = 0
            for i in range(len(blobs)):
            #目标区域找到的颜色块（线段块）可能不止一个，找到最大的一个，作为本区域内的目标直线
                if blobs[i].pixels() > most_pixels:
                    most_pixels = blobs[i].pixels()
                    #merged_blobs[i][4]是这个颜色块的像素总数，如果此颜色块像素总数大于
                    #most_pixels，则把本区域作为像素总数最大的颜色块。更新most_pixels和largest_blob
                    largest_blob = i
            # 在色块周围画一个矩形。
            img.draw_rectangle(blobs[largest_blob].rect())
            # 将此区域的像素数最大的颜色块画矩形和十字形标记出来
            img.draw_cross(blobs[largest_blob].cx(),
                           blobs[largest_blob].cy())
            redLineFlag = 1 # 检测到红线更新标志位
        if redLineFlag == 1: # 检测后对应位置为1
           roiRegion.insert(counterRoi-1, 1)
        else: roiRegion.insert(counterRoi-1, 0) # 未检测对应位置为0
    trafficResult = trafficLogic(roiRegion) # 道路口逻辑判断
    return trafficResult


# 处理下位机信息【TODO: 用于处理下位机反馈的消息】
def backUp(task):
    if task == 1: # 倒车
        pass


# 事件处理与下位机通信【TODO: 丰富事件库】
def message(pos, posResult, trafficResult):
    if pos == 1: # 去1号病房不需要检测数字
        if trafficResult == 2: # 十字路口
            print("向左转进入1号病房")
    elif pos == 2: # 去2号病房不需要检测数字
        if trafficResult == 2: # 十字路口
            print("向左转进入2号病房")
    if posResult == 0: # 未能检测到目标
        if trafficResult == 0: # 错误码：异常情况
            pass
        elif trafficResult == 1: # 沿线前进
            print("沿线前进")
        elif trafficResult == 2: # 十字路口
            print("十字路口")
        elif trafficResult == 3: # 正对T字路口
            pass
        elif trafficResult == 4: # T字路口向左
            print("T字路口向左")
        elif trafficResult == 5: # T字路口向右
            print("T字路口向右")
        elif trafficResult == 6: # 已经到达道路尽头
            print("已经到达道路镜头")
        elif trafficResult == 7: # 跑偏或驶离赛道
            pass

    elif posResult == 1: # 检测目标在左侧
        if trafficResult == 0: # 错误码：异常情况
            pass
        elif trafficResult == 1: # 沿线前进
            pass
        elif trafficResult == 2: # 十字路口
            print("十字路口左转弯")
        elif trafficResult == 3: # 正对T字路口
            print("T字路口左转弯")

    elif posResult == 2: # 检测目标在右侧
        if trafficResult == 0: # 错误码：异常情况
            pass
        elif trafficResult == 1: # 沿线前进
            pass
        elif trafficResult == 2: # 十字路口
            print("十字路口右转弯")
        elif trafficResult == 3: # 正对T字路口
            print("T字路口右转弯")


    #if trafficResult == 7:
        #VISIONWORK = bytearray([0x2c,0x01,0x5b]) # 视觉开始工作
        #mess = VISIONWORK
    #elif trafficResult == 2:
        #VISIONPAUSE = bytearray([0x2c,0x02,0x5b]) # 视觉停止工作
        #mess = VISIONPAUSE
    #elif trafficResult == 3:
        #TEMPLATE_OK = bytearray([0x2c,0x03,0x5b]) # 完成模板匹配
        #mess = TEMPLATE_OK
    #elif trafficResult == 4:
        #TEMPLATE_FAIL = bytearray([0x2c,0x04,0x5b]) # 模板匹配失败
        #mess = TEMPLATE_FAIL
    #elif trafficResult == 5:
        #CROSSROAD_OK = bytearray([0x2c,0x05,0x5b]) # 完成路口检测
        #mess = CROSSROAD_OK
    #elif trafficResult == 6:
        #CROSSROAD_FAIL = bytearray([0x2c,0x06,0x5b]) # 路口检测失败
        #mess = CROSSROAD_FAIL
    #elif trafficResult == 1:
        #CROSSROAD_STREET = bytearray([0x2c,0x07,0x5b]) # 路口直行
        #mess = CROSSROAD_STREET
    #elif trafficResult == 8:
        #CROSSROAD_LEFT = bytearray([0x2c,0x08,0x5b]) # 路口左转
        #mess = CROSSROAD_LEFT
    #elif trafficResult == 9:
        #CROSSROAD_RIGHT = bytearray([0x2c,0x09,0x5b]) # 路口右转
        #mess = CROSSROAD_RIGHT
    #else: # 未能匹配上述事件
        #Fail = bytearray([0x2c,0x10,0x5b]) # 错误
        #mess = Fail
    #uart.write(mess)
    #return 1 # 允许检测



# 程序主循环
while(True):
    for i in range(100):
        clock.tick()
        if uart.any():
            mes = uart.readline().decode().strip().split(',') # 对接收的数据进行解码

        img = sensor.snapshot()
        #img.lens_corr(1.1) # 相机畸变矫正
        lastValue = crossRoadSeparate(img, red_threshold, lastValue) # 道路左右侧分界检测(time ~ 0.8FPS)

        # 1、获取送药任务
        if dectectionFlag:
            for i in [1,2,8,4,6,5,3,7]: # 将易误识别数字顺序进行了调整(主要针对8、6、5、3)
              if templateMatch(img, dectectionFlag, i):
                   targetNum = i # 获得目标数字为i
                   dectectionFlag = False # 数字指令已接受
                   print("目标病房号已明确，为:",targetNum)
                   break
            continue
        # 小车巡线
        deflectionAngle = lineFlowing(img)

        # 2、进行数字模板匹配
        matchResult = templateMatch(img, dectectionFlag, targetNum)
        if matchResult: # 匹配目标数字
            posResult = turnLogic(matchResult, lastValue) # 获取转向结果
            print("判断结果为:",posResult)
            #print("已检测到病房号:",targetNum)

        ## 3、检测红线路口
        trafficResult = crossRoadDetection(img, red_threshold)

        # 4、传递信息给下位机
        message(targetNum, posResult, trafficResult)

        # 5、绘制图像(用于调试可视化)
        drowWindows(img)

        # 重置变量
        posResult = 0
        trafficResult = 0
        #print(clock.fps()) # 显示耗时
