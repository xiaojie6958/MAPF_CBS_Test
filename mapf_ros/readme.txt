## Dependicies
sudo apt-get install ros-melodic-costmap-2d
sudo apt-get install ros-melodic-move-base-msgs
sudo apt-get install ros-melodic-rviz
sudo apt-get install libtinyxml-dev

## learn how to pull


## How to run

roslaunch mapf_ros my_cbs_ros.launch 
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[6,19]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[4,10]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[19,17]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[28,27]\"}'"//A

rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[29,54]\"}'"//workss
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[57,26]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[11,56]\"}'"

rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[11,54]\"}'"//workss
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[55,57]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[22,26]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[27,59]\"}'"



rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[10,21]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[14,21]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[8,54]\"}'"  //

rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[15,30]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[35,48]\"}'"
rostopic pub /setOrder std_msgs/String "data: '{\"end\": \"[{8,20},{1,4}]\"}'"
1 means: Point-002
4 means: Point-005

1. 发布后 自动分配车辆，进行规划
2. 到达任务后，自动规划回到 停车点

## Test rostopic
1. rostopic list
2. rostopic echo /setOrder



## TODO

1. sudo apt install libtinyxml-dev # parser 移除 ,使用 tinyxml2(ubuntu18 native installed)
2. sudo apt-get install ros-melodic-rviz ， docker added rviz

5. 流程划分
  - 任务分配（plug in）
  - 路径规划
  - 冲突消解
6. 输入和输出


- step1:parser ,ros和非ros部分拆解
  - parse ros部分分割 done
- step2:清理无用代码
- step3:remove dijkstra.hpp
- step4:my_cbs ---> singlePlanner
  - 模块作用明确
  - 移除 vector 放到外部，本部分仅作为 Single Agent's Planner
- step4:my_cbs_ros拆解
  - step4.1 结构划分
    - ros-layer
    - process-layer
    - conflcit ---> MultiPlannerManager
    - add SinglePlannerManager
  - step4.2 conflict 拆解 // comment ros结合太紧 延后拆解
  - step4.3 AssignTask 拆解， 在GRPC中，又调度系统直接提供结果
    - assign 之后 ，再进行 SinglePlanner的实例化
    - question： 
      - 如何管理 Agent 类？ Agent 和 Dispatch 、 singlePlanner 以及 MultiPlannerManager 的关系
      - 同理，关于 Task 的管理，Order，车库 ... 
      - 整个调度流程中数据管理中台的设计？
  - 当前 app 中是模拟了一个 回到从车库出发，再回到车库的场景 ...
    - def Enviroments(Related to Tasks)
      - Agent: 记录车辆当前状态 ... 
      - Task
      - SinglePlanner：combine with Agent, Task, Order, Garage
      - Garage
      - 其它组件：算完之后，如何可视化
  - 增加一个类，实现 ros和非ros的剥离 [going]
    - my_cbs_new.cc && my_cbs_new.h, 用于存放 独立的cbs数据
    - 和ros相关的放在外面 
      - ros 作为接口存在
  - TBD
    - load map
      - tinyxml  ---> tinyxml2
      - 加载混合地图
      - c++ define datastructure ---> pb structure
    - add agent[id]
    - add constrain
    - add task[start, end]
    - order format 修改 ...


## ChangeLog

- 来自 Zhuhu's code 240803
