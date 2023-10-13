# Update log
1.柔和变速转弯
修改位置：./control/pathtrack/src/pathtrack.cpp:296-310 最大速度由私有ros参数turn_speed_max给出
2.取车依据车轮位置提前减速停车，减速距离更短提高效率
修改位置：./global_plan_sim/src/localpathplan.cpp:697-714
相关数据支持添加:./global_plan_sim/src/localpathplan.cpp:26 添加车轮识别结果的消息订阅
                ./global_plan_sim/src/localpathplan.cpp:76 车轮识别结果回调函数声明
                ./global_plan_sim/src/localpathplan.cpp:64 车轮识别结果的消息订阅初始化
                ./global_plan_sim/src/localpathplan.cpp:42 车轮识别结果存储变量声明
                ./global_plan_sim/src/localpathplan.cpp:47 车轮识别结果存储变量初始化
                ./global_plan_sim/src/localpathplan.cpp:103-110 车轮识别结果回调函数实现
3.激光雷达信号处理
新增代码：./sensor/launch.c200_4_new.launch 为外侧激光雷达启动数据转换节点，发布激光雷达到车体的左边转换关系
         ./laserscan_check/src/laserscan_check_around.cpp 由./laserscan_check/launch/run_test_new.launch启动
                                                          将接收传感器信息生成车体坐标系下的点云信息并发布
4.激光雷达纠偏
修改位置：./laserscan_check/src/laserscan_check_angle.cpp:整体修改 
            内侧线激光，在进车和退车过程中均起作用，外侧线激光只在进车中起作用
            该节点对内外激光雷达共8个分别进行滤波处理，得到商品车距离左右激光雷达的最小的左边y值的差，并左右相加
            以wheel_err消息输出，wheel_err共包含4个数，分别是内侧激光后方偏差，内侧激光前方偏差，外侧激光后方偏差，外侧激光前方偏差
相关数据执行：./control/pathtrack/src/pathtrack.cpp:591-619 纠偏执行代码
             ./control/pathtrack/src/pathtrack.cpp:99 数据回调函数声明
             ./control/pathtrack/src/pathtrack.cpp:148-158 数据回调函数实现
             ./control/pathtrack/src/pathtrack.cpp:26 消息订阅变量声明
             ./control/pathtrack/src/pathtrack.cpp:81 消息订阅变量初始化
             ./control/pathtrack/src/pathtrack.cpp:54-55 数据存储
5.激光雷达避障
新增代码：./perception/cloud_preprocess/src/laser_radar_obs.cpp
         根据localpath和车体控制指令修改避障范围
         自转为圆形避障区域，其他运动模式为矩形避障区域，在rviz中显示白色框或圆形
         检查四周20cm的范围，运动方向上延长7m，提前2.5m停车
         在拐弯处检测到路径拐点的车辆边框外延0.5m的位置，保证安全的同时防止避障误触
         程序只输出范围内经过滤波和聚类后距离车边框的最近距离 以rosparam形式输出
相关数据执行和显示：./control/pathtrack/src/pathtrack.cpp:576 读取最近距离，与多线激光雷达获取的最近距离取最小
                                                             生成obstacle_speedlimit，限制速度
                  ./control/pathtrack/src/pathtrack.cpp:296-310 自转同样应用obstacle_speedlimit提供的速度限制
                                                                实现自转避障
                  ./global_plan_sim/src/panel/global_plan_sim_panel.cpp:191 读取最近距离参数与多线激光雷达获取的最近距离
                                                                            取最小，显示在rviz上
