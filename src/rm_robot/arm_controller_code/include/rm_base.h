#ifndef RM_BASE_H
#define RM_BASE_H
//////////////////////////////////////////////////////////////////////////////////
//睿尔曼智能科技有限公司        Author:Dong Qinpeng
//创建日期:2022/08/23
//版本：V4.0
//版权所有，盗版必究。
//Copyright(C) 睿尔曼智能科技有限公司
//All rights reserved
//文档说明：该文档声明了机械臂所有操作接口
//////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif
#include "rm_base_global.h"
#include "rm_define.h"


///
/// \brief RM_API_Init              API初始化
/// \param dev_mode                 目标设备型号使用宏定义ARM_65/ARM_75/ARM_63_1/ARM_63_2
/// \param pCallback                用于接收透传接口回调函数, 不需要可以传入NULL
///  RM_APISHARED_EXPORT
///
RM_BASESHARED_EXPORT int RM_API_Init(int devMode, RM_Callback pCallback);

///
/// \brief RM_API_UnInit              API反初始化
///  RM_APISHARED_EXPORT
///
RM_BASESHARED_EXPORT int RM_API_UnInit();

///
/// \brief API_Version              查询API版本信息
/// \return                         API版本号
///
RM_BASESHARED_EXPORT char * API_Version();

///
/// \brief Arm_Socket_Start         连接机械臂
/// \param arm_ip                   IP地址
/// \param arm_port                 端口号
/// \param recv_timeout             Socket连接超时时间
/// \return                         成功返回：Socket 句柄。失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT SOCKHANDLE Arm_Socket_Start(char* arm_ip, int arm_port, int recv_timeout);

///
/// \brief Arm_Socket_Close         关闭与机械臂的Socket连接
/// \param ArmSocket                Socket句柄
///
RM_BASESHARED_EXPORT void Arm_Socket_Close(SOCKHANDLE ArmSocket);

///
/// \brief Arm_Sockrt_State         查询机械臂连接状态
/// \param ArmSocket                Socket句柄
/// \return                         正常返回：0。失败返回:错误码, define.h查询
///
RM_BASESHARED_EXPORT int Arm_Socket_State(SOCKHANDLE ArmSocket);

///
/// \brief Set_Joint_Speed 设置关节最大速度
/// \param ArmSocket socket句柄
/// \param joint_num 关节序号，1~7
/// \param speed 关节转速，单位：°/s
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_Speed(SOCKHANDLE ArmSocket, byte joint_num, float speed, bool block);

///
/// \brief Set_Joint_Acc 设置关节最大加速度
/// \param ArmSocket socket句柄
/// \param joint_num 关节序号，1~7
/// \param acc 关节转速，单位：°/s²
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_Acc(SOCKHANDLE ArmSocket, byte joint_num, float acc, bool block);

///
/// \brief Set_Joint_Min_Pos 设置关节最小限位
/// \param ArmSocket socket句柄
/// \param joint_num 关节序号，1~7
/// \param joint 关节最小位置，单位：°
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_Min_Pos(SOCKHANDLE ArmSocket, byte joint_num, float joint, bool block);

///
/// \brief Set_Joint_Max_Pos 设置关节最大限位
/// \param ArmSocket socket句柄
/// \param joint_num 关节序号，1~7
/// \param joint 关节最大位置，单位：°
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_Max_Pos(SOCKHANDLE ArmSocket, byte joint_num, float joint, bool block);

///
/// \brief Set_Joint_EN_State 设置关节使能状态
/// \param ArmSocket socket句柄
/// \param joint_num 关节序号，1~7
/// \param state true-上使能，false-掉使能
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_EN_State(SOCKHANDLE ArmSocket, byte joint_num, bool state, bool block);

///
/// \brief Set_Joint_Zero_Pos 将当前位置设置为关节零位
/// \param ArmSocket socket句柄
/// \param joint_num 关节序号，1~7
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_Zero_Pos(SOCKHANDLE ArmSocket, byte joint_num, bool block);

///
/// \brief Set_Joint_Err_Clear 清除关节错误代码
/// \param ArmSocket socket句柄
/// \param joint_num 关节序号，1~7
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_Err_Clear(SOCKHANDLE ArmSocket, byte joint_num, bool block);

///
/// \brief Get_Joint_Speed 查询关节最大速度
/// \param ArmSocket socket句柄
/// \param speed 关节1~7转速数组，单位：°/s
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Speed(SOCKHANDLE ArmSocket, float *speed);

///
/// \brief Get_Joint_Acc 查询关节最大加速度
/// \param ArmSocket socket句柄
/// \param acc 关节1~7加速度数组，单位：°/s²
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Acc(SOCKHANDLE ArmSocket, float *acc);

///
/// \brief Get_Joint_Min_Pos 获取关节最小限位
/// \param ArmSocket socket句柄
/// \param min_joint 关节1~7最小位置数组，单位：°
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Min_Pos(SOCKHANDLE ArmSocket, float *min_joint);

///
/// \brief Get_Joint_Max_Pos 获取关节最大限位
/// \param ArmSocket socket句柄
/// \param max_joint 关节1~7最大位置数组，单位：°
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Max_Pos(SOCKHANDLE ArmSocket, float *max_joint);

///
/// \brief Get_Joint_EN_State 获取关节使能状态
/// \param ArmSocket socket句柄
/// \param state 关节1~7使能状态数组，1-使能状态，0-掉使能状态
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_EN_State(SOCKHANDLE ArmSocket, byte *state);

///
/// \brief Get_Joint_Err_Flag 获取关节Err Flag
/// \param ArmSocket socket句柄
/// \param state 关节错误码(请参考api文档中的关节错误码)
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Err_Flag(SOCKHANDLE ArmSocket, uint16_t *state);

///
/// \brief Get_Joint_Software_Version       查询关节软件版本号
/// \param ArmSocket socket句柄
/// \param version                          获取到的关节软件版本号
/// \return                                 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Software_Version(SOCKHANDLE ArmSocket, float* version);

///
/// \brief Set_Arm_Line_Speed 设置机械臂末端最大线速度
/// \param ArmSocket socket句柄
/// \param speed 末端最大线速度，单位m/s
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Line_Speed(SOCKHANDLE ArmSocket, float speed, bool block);

///
/// \brief Set_Arm_Line_Acc 设置机械臂末端最大线加速度
/// \param ArmSocket socket句柄
/// \param acc 末端最大线加速度，单位m/s^2
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Line_Acc(SOCKHANDLE ArmSocket, float acc, bool block);

/// \brief Set_Arm_Angular_Speed 设置机械臂末端最大角速度
/// \param ArmSocket socket句柄
/// \param speed 末端最大角速度，单位rad/s
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Angular_Speed(SOCKHANDLE ArmSocket, float speed, bool block);

///
/// \brief Set_Arm_Angular_Acc 设置机械臂末端最大角加速度
/// \param ArmSocket socket句柄
/// \param acc 末端最大角加速度，单位rad/s^2
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Angular_Acc(SOCKHANDLE ArmSocket, float acc, bool block);

///
/// \brief Get_Arm_Line_Speed 获取机械臂末端最大线速度
/// \param ArmSocket socket句柄
/// \param speed 末端最大线速度，单位m/s
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Line_Speed(SOCKHANDLE ArmSocket, float *speed);

///
/// \brief Get_Arm_Line_Acc 获取机械臂末端最大线加速度
/// \param ArmSocket socket句柄
/// \param acc 末端最大线加速度，单位m/s^2
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Line_Acc(SOCKHANDLE ArmSocket, float *acc);

///
/// \brief Get_Arm_Angular_Speed 获取机械臂末端最大角速度
/// \param ArmSocket socket句柄
/// \param speed 末端最大角速度，单位rad/s
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Angular_Speed(SOCKHANDLE ArmSocket, float *speed);

///
/// \brief Get_Arm_Angular_Acc 获取机械臂末端最大角加速度
/// \param ArmSocket socket句柄
/// \param acc 末端最大角加速度，单位rad/s^2
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Angular_Acc(SOCKHANDLE ArmSocket, float *acc);

///
/// \brief Set_Arm_Tip_Init 设置机械臂末端参数为默认值
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Tip_Init(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Set_Collision_Stage 设置机械臂动力学碰撞检测等级
/// \param ArmSocket socket句柄
/// \param stage 等级：0~8，0-无碰撞，8-碰撞最灵敏
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Collision_Stage(SOCKHANDLE ArmSocket, int stage, bool block);

///
/// \brief Get_Collision_Stage 查询碰撞防护等级
/// \param ArmSocket socket句柄
/// \param stage    传出参数，表示等级，范围：0~8
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Collision_Stage(SOCKHANDLE ArmSocket, int *stage);

///
/// \brief Set_DH_Data 该函数用于重新设置DH参数，一般在标定后使用
/// \param ArmSocket socket句柄
/// \param lsb 相关DH参数，单位：mm
/// \param lse 相关DH参数，单位：mm
/// \param lew 相关DH参数，单位：mm
/// \param lwt 相关DH参数，单位：mm
/// \param d3  相关DH参数，单位：mm
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_DH_Data(SOCKHANDLE ArmSocket, float lsb, float lse, float lew, float lwt_, float d3, bool block);

///
/// \brief Get_DH_Data 该函数用于获取DH参数，一般在标定后使用
/// \param ArmSocket socket句柄
/// \param lsb 获取到的DH参数，单位：mm
/// \param lse 获取到的DH参数，单位：mm
/// \param lew 获取到的DH参数，单位：mm
/// \param lwt 获取到的DH参数，单位：mm
/// \param d3  获取到的DH参数，单位：mm
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_DH_Data(SOCKHANDLE ArmSocket, float* lsb, float* lse, float* lew, float* lwt_, float* d3);

///
/// \brief Set_Joint_Zero_Offset 该函数用于设置机械臂各关节零位补偿角度，一般在对机械臂零位进行标定后调用该函数
/// \param ArmSocket socket句柄
/// \param offset 关节1~6的零位补偿角度数组, 单位：度
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Joint_Zero_Offset(SOCKHANDLE ArmSocket, const float *offset, bool block);

///
/// \brief          Set_Arm_Dynamic_Parm 重新设置机械臂动力学参数
/// \param ArmSocket socket句柄
/// \param parm     设置机械臂动力学参数，
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0       -成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Dynamic_Parm(SOCKHANDLE ArmSocket, const float* parm, bool block);

///
/// \brief Get_Tool_Software_Version        查询末端接口板软件版本号
/// \param ArmSocket socket句柄
/// \param version                          查询到的末端接口板软件版本号
/// \return                                 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Tool_Software_Version(SOCKHANDLE ArmSocket, int* version);

/// \brief Set_Arm_Servo_State 打开或者关闭控制器对机械臂伺服状态查询
/// \param ArmSocket socket句柄
/// \param cmd true-打开，  false-关闭
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Servo_State(SOCKHANDLE ArmSocket, bool cmd, bool block);

///
/// \brief Auto_Set_Tool_Frame 六点法自动设置工具坐标系 标记点位
/// \param ArmSocket socket句柄
/// \param point_num 1~6代表6个标定点
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 说明：控制器只能存储十个工具，超过十个控制器不予响应，请在标定前查询已有工具
RM_BASESHARED_EXPORT int Auto_Set_Tool_Frame(SOCKHANDLE ArmSocket, byte point_num, bool block);

///
/// \brief Generate_Auto_Tool_Frame 六点法自动设置工具坐标系 提交
/// \param ArmSocket socket句柄
/// \param name  工具坐标系名称，不能超过十个字节。
/// \param payload 新工具执行末端负载重量  单位kg
/// \param x 新工具执行末端负载位置 位置x 单位mm
/// \param y 新工具执行末端负载位置 位置y 单位mm
/// \param z 新工具执行末端负载位置 位置z 单位mm
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Generate_Auto_Tool_Frame(SOCKHANDLE ArmSocket, const char *name,float payload,float x,float y,float z, bool block);

///
/// \brief Manual_Set_Tool_Frame 手动设置工具坐标系
/// \param ArmSocket socket句柄
/// \param name 工具坐标系名称，不能超过十个字节。
/// \param pose 新工具执行末端相对于机械臂法兰中心的位姿
/// \param payload 新工具执行末端负载重量  单位kg
/// \param x 新工具执行末端负载位置 位置x 单位m
/// \param y 新工具执行末端负载位置 位置y 单位m
/// \param z 新工具执行末端负载位置 位置z 单位m
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 控制器只能存储十个工具，超过十个控制器不予响应，请在标定前查询已有工具
RM_BASESHARED_EXPORT int Manual_Set_Tool_Frame(SOCKHANDLE ArmSocket, const char *name, POSE pose,float payload,float x,float y,float z, bool block);

///
/// \brief Change_Tool_Frame 切换当前工具坐标系
/// \param ArmSocket socket句柄
/// \param name 目标工具坐标系名称
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Change_Tool_Frame(SOCKHANDLE ArmSocket, const char *name, bool block);

///
/// \brief Delete_Tool_Frame 删除指定工具坐标系
/// \param ArmSocket socket句柄
/// \param name 要删除的工具坐标系名称
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 备注：删除坐标系后，机械臂将切换到机械臂法兰末端工具坐标系
RM_BASESHARED_EXPORT int Delete_Tool_Frame(SOCKHANDLE ArmSocket, const char *name, bool block);

///
/// \brief Set_Payload 设置末端负载质量和质心
/// \param ArmSocket socket句柄
/// \param payload 负载质量，单位：g，最高不超过5000g
/// \param cx 末端负载质心位置，单位：m
/// \param cy 末端负载质心位置，单位：m
/// \param cz 末端负载质心位置，单位：m
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Payload(SOCKHANDLE ArmSocket, int payload, float cx, float cy, float cz, bool block);

///
/// \brief Set_None_Payload 设置末端无负载
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_None_Payload(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Get_Current_Tool_Frame 获取当前工具坐标系
/// \param ArmSocket socket句柄
/// \param tool 返回的坐标系
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Current_Tool_Frame(SOCKHANDLE ArmSocket, FRAME *tool);

///
/// \brief Get_Given_Tool_Frame 获取指定工具坐标系
/// \param ArmSocket socket句柄
/// \param name 指定的工具名称
/// \param tool 返回的工具参数
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Given_Tool_Frame(SOCKHANDLE ArmSocket, const char *name, FRAME *tool);

///
/// \brief Get_All_Tool_Frame 获取所有工具坐标系名称
/// \param ArmSocket socket句柄
/// \param names 返回的工具名称数组
/// \param len 返回工具数量
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_All_Tool_Frame(SOCKHANDLE ArmSocket, FRAME_NAME *names, int *len);

/// \brief Auto_Set_Work_Frame 三点法自动设置工作坐标系
/// \param ArmSocket socket句柄
/// \param name 工作坐标系名称，不能超过十个字节。
/// \param point_num 1~3代表3个标定点，依次为原点、X轴一点、Y轴一点，4代表生成坐标系。
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 控制器只能存储十个工作坐标系，超过十个控制器不予响应，请在标定前查询已有工作坐标系
RM_BASESHARED_EXPORT int Auto_Set_Work_Frame(SOCKHANDLE ArmSocket, const char *name, byte point_num, bool block);

///
/// \brief Manual_Set_Work_Frame 手动设置工作坐标系
/// \param ArmSocket socket句柄
/// \param name 工作坐标系名称，不能超过十个字节。
/// \param pose 新工作坐标系相对于基坐标系的位姿
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 控制器只能存储十个工作坐标系，超过十个控制器不予响应，请在标定前查询已有工作坐标系
RM_BASESHARED_EXPORT int Manual_Set_Work_Frame(SOCKHANDLE ArmSocket, const char *name, POSE pose, bool block);

///
/// \brief Change_Work_Frame 切换当前工作坐标系
/// \param ArmSocket socket句柄
/// \param name 目标工作坐标系名称
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Change_Work_Frame(SOCKHANDLE ArmSocket, const char *name, bool block);


///
/// \brief Delete_Work_Frame 删除指定工作坐标系
/// \param ArmSocket socket句柄
/// \param name 要删除的工具坐标系名称
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 备注：删除坐标系后，机械臂将切换到机械臂基坐标系
RM_BASESHARED_EXPORT int Delete_Work_Frame(SOCKHANDLE ArmSocket, const char *name, bool block);

////
/// \brief Get_Current_Work_Frame 获取当前工作坐标系
/// \param ArmSocket socket句柄
/// \param frame 返回的坐标系
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Current_Work_Frame(SOCKHANDLE ArmSocket, FRAME *frame);

///
/// \brief Get_Given_Work_Frame 获取指定工作坐标系
/// \param ArmSocket socket句柄
/// \param name 指定的工作坐标系名称
/// \param pose 获取到的位姿
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Given_Work_Frame(SOCKHANDLE ArmSocket, const char *name, POSE *pose);

///
/// \brief Get_All_Work_Frame 获取所有工作坐标系名称
/// \param ArmSocket socket句柄
/// \param names 返回的工作坐标系名称字符数组
/// \param len 返回的工作坐标系名称长度
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_All_Work_Frame(SOCKHANDLE ArmSocket, FRAME_NAME *names, int *len);

///
/// \brief Get_Current_Arm_State 获取机械臂当前状态
/// \param ArmSocket socket句柄
/// \param joint 关节1~6角度数组
/// \param pose 机械臂当前位姿
/// \param Arm_Err 机械臂运行错误代码
/// \param Sys_Err 控制器错误代码
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Current_Arm_State(SOCKHANDLE ArmSocket, float *joint, POSE *pose, uint16_t *Arm_Err, uint16_t *Sys_Err);

///
/// \brief Get_Joint_Temperature 获取关节当前温度
/// \param ArmSocket socket句柄
/// \param temperature 关节1~7温度数组
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Temperature(SOCKHANDLE ArmSocket, float *temperature);

///
/// \brief Get_Joint_Current 获取关节当前电流
/// \param ArmSocket socket句柄
/// \param current 关节1~7电流数组
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Current(SOCKHANDLE ArmSocket, float *current);

///
/// \brief Get_Joint_Voltage 获取关节当前电压
/// \param ArmSocket socket句柄
/// \param voltage 关节1~7电压数组
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Voltage(SOCKHANDLE ArmSocket, float *voltage);

///
/// \brief Get_Joint_Degree 获取当前关节角度
/// \param ArmSocket socket句柄
/// \param joint 当前7个关节的角度数组
/// \return
///
RM_BASESHARED_EXPORT int Get_Joint_Degree (SOCKHANDLE ArmSocket, float *joint);

///
/// \brief Get_Arm_All_State 获取机械臂所有状态信息
/// \param ArmSocket socket句柄
/// \param joint_state 存储机械臂信息的结构体
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_All_State(SOCKHANDLE ArmSocket, JOINT_STATE *joint_state);

///
/// \brief Get_Arm_Plan_Num             查询规划计数
/// \param ArmSocket                    socket句柄
/// \param plan                         查询到的规划计数
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Plan_Num(SOCKHANDLE ArmSocket, int* plan);

///
/// \brief Set_Arm_Init_Pose 设置机械臂的初始位置角度
/// \param ArmSocket socket句柄
/// \param target 机械臂初始位置关节角度数组
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Init_Pose(SOCKHANDLE ArmSocket, const float *target, bool block);

///
/// \brief Get_Arm_Init_Pose 获取机械臂初始位置角度
/// \param ArmSocket socket句柄
/// \param joint 机械臂初始位置关节角度数组
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Init_Pose(SOCKHANDLE ArmSocket, float *joint);

///
/// \brief Set_Install_Pose     设置安装方式参数
/// \param                      ArmSocket socket句柄
/// \param x                    旋转角，单位 °
/// \param y                    俯仰角，单位 °
/// \param z                    方位角，单位 °
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                     0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Install_Pose(SOCKHANDLE ArmSocket, float x, float y, float z, bool block);

///
/// \brief Get_Install_Pose     获取安装方式参数
/// \param                      ArmSocket socket句柄
/// \param x                    旋转角(out)
/// \param y                    俯仰角(out)
/// \param z                    方位角(out)
/// \return                     0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Install_Pose(SOCKHANDLE ArmSocket, float *fx, float *fy, float *fz);

///
/// \brief Movej_Cmd 关节空间运动
/// \param ArmSocket socket句柄
/// \param joint 目标关节1~7角度数组
/// \param v 速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比
/// \param r 轨迹交融半径，目前默认0。
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂到达位置或者规划失败
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Movej_Cmd(SOCKHANDLE ArmSocket, const float *joint, byte v, float r, bool block);

///
/// \brief Movel_Cmd 笛卡尔空间直线运动
/// \param ArmSocket socket句柄
/// \param pose 目标位姿,位置单位：米，姿态单位：弧度
/// \param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
/// \param r 轨迹交融半径，目前默认0。
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂到达位置或者规划失败
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Movel_Cmd(SOCKHANDLE ArmSocket, POSE pose, byte v, float r, bool block);

///
/// \brief Movec_Cmd 笛卡尔空间圆弧运动
/// \param ArmSocket socket句柄
/// \param pose_via 中间点位姿，位置单位：米，姿态单位：弧度
/// \param pose_to 终点位姿
/// \param v 速度比例1~100，即规划速度和加速度占机械臂末端最大角速度和角加速度的百分比
/// \param r 轨迹交融半径，目前默认0。
/// \param loop 规划圈数，目前默认0.
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂到达位置或者规划失败
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Movec_Cmd(SOCKHANDLE ArmSocket, POSE pose_via, POSE pose_to, byte v, float r, byte loop, bool block);

///
/// \brief Movej_CANFD 角度不经规划，直接通过CANFD透传给机械臂
/// \param ArmSocket socket句柄
/// \param joint 关节1~7目标角度数组
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 只要控制器运行正常并且目标角度在可达范围内，机械臂立即返回成功指令，此时机械臂可能仍在运行；
/// 若有错误，立即返回失败指令。
RM_BASESHARED_EXPORT int Movej_CANFD(SOCKHANDLE ArmSocket, const float *joint);

///
/// \brief Movep_CANFD 位姿不经规划，直接通过CANFD透传给机械臂
/// \param ArmSocket socket句柄
/// \param pose 位姿
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Movep_CANFD(SOCKHANDLE ArmSocket, POSE pose);

///
/// \brief MoveRotate_Cmd                   计算环绕运动位姿并按照结果运动
/// \param ArmSocket                    socket句柄
/// \param rotateAxis                   旋转轴: 1:x轴, 2:y轴, 3:z轴
/// \param rotateAngle                  旋转角度: 旋转角度, 单位(度)
/// \param choose_axis                  指定计算时使用的坐标系
/// \param v                            速度
/// \param r                            交融半径
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
RM_BASESHARED_EXPORT int MoveRotate_Cmd(SOCKHANDLE ArmSocket, int rotateAxis, float rotateAngle, Pose choose_axis, byte v, float r,  bool block);

///
/// \brief MoveCartesianTool_Cmd           沿工具端位姿移动
/// \param Joint_Cur                当前关节角度
/// \param movelengthx:             沿X轴移动长度，米为单位
/// \param movelengthy:             沿Y轴移动长度，米为单位
/// \param movelengthz:             沿Z轴移动长度，米为单位
/// \param m_dev                    机械臂型号
/// \param v                        速度
/// \param r                        交融半径
/// \param block                    RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂到达位置或者规划失败
/// \return                         0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int MoveCartesianTool_Cmd(SOCKHANDLE ArmSocket, float *Joint_Cur, float movelengthx, float movelengthy, float movelengthz, int m_dev, byte v, float r,  bool block);

///
/// \brief Move_Stop_Cmd 突发状况 机械臂以最快速度急停，轨迹不可恢复
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Move_Stop_Cmd(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Move_Pause_Cmd 轨迹暂停，暂停在规划轨迹上，轨迹可恢复
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Move_Pause_Cmd(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Move_Continue_Cmd 轨迹暂停后，继续当前轨迹运动
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Move_Continue_Cmd(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Clear_Current_Trajectory 清除当前轨迹，必须在暂停后使用，否则机械臂会发生意外！！！！
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Clear_Current_Trajectory(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Clear_All_Trajectory 清除所有轨迹，必须在暂停后使用，否则机械臂会发生意外！！！！
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Clear_All_Trajectory(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Movej_P_Cmd 该函数用于关节空间运动到目标位姿
/// \param ArmSocket socket句柄
/// \param pose: 目标位姿，位置单位：米，姿态单位：弧度。注意：该目标位姿必须是机械臂末端末端法兰中心基于基坐标系的位姿！！
/// \param v: 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
/// \param r: 轨迹交融半径，目前默认0。
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂到达位置或者规划失败
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Movej_P_Cmd(SOCKHANDLE ArmSocket, POSE pose, byte v, float r, bool block);


///
/// \brief Joint_Teach_Cmd 关节示教
/// \param ArmSocket socket句柄
/// \param num 示教关节的序号，1~7
/// \param direction 示教方向，0-负方向，1-正方向
/// \param v 速度比例1~100，即规划速度和加速度占关节最大线转速和加速度的百分比
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Joint_Teach_Cmd(SOCKHANDLE ArmSocket, byte num, byte direction, byte v, bool block);

///
/// \brief Pos_Teach_Cmd 当前工作坐标系下，笛卡尔空间位置示教
/// \param ArmSocket socket句柄
/// \param type 示教类型
/// \param direction 示教方向，0-负方向，1-正方向
/// \param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Pos_Teach_Cmd(SOCKHANDLE ArmSocket, POS_TEACH_MODES type, byte direction, byte v, bool block);

///
/// \brief Ort_Teach_Cmd 当前工作坐标系下，笛卡尔空间姿态示教
/// \param ArmSocket socket句柄
/// \param type 示教类型
/// \param direction 示教方向，0-负方向，1-正方向
/// \param v 速度比例1~100，即规划速度和加速度占机械臂末端最大角速度和角加速度的百分比
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Ort_Teach_Cmd(SOCKHANDLE ArmSocket, ORT_TEACH_MODES type, byte direction, byte v, bool block);

///
/// \brief Teach_Stop_Cmd 示教停止
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Teach_Stop_Cmd(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Joint_Step_Cmd 关节步进
/// \param ArmSocket socket句柄
/// \param num 关节序号，1~7
/// \param step 步进的角度，
/// \param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂返回失败或者到达位置指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Joint_Step_Cmd(SOCKHANDLE ArmSocket, byte num, float step, byte v, bool block);

///
/// \brief Pos_Step_Cmd 当前工作坐标系下，位置步进
/// \param ArmSocket socket句柄
/// \param type 示教类型
/// \param step 步进的距离，单位m，精确到0.001mm
/// \param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂返回失败或者到达位置指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Pos_Step_Cmd(SOCKHANDLE ArmSocket, POS_TEACH_MODES type, float step, byte v, bool block);

///
/// \brief Ort_Step_Cmd 当前工作坐标系下，姿态步进
/// \param ArmSocket socket句柄
/// \param type 示教类型
/// \param step 步进的弧度，单位rad，精确到0.001rad
/// \param v 速度比例1~100，即规划速度和加速度占机械臂末端最大线速度和线加速度的百分比
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待机械臂返回失败或者到达位置指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Ort_Step_Cmd(SOCKHANDLE ArmSocket, ORT_TEACH_MODES type, float step, byte v, bool block);

///
/// \brief Get_Controller_State 获取控制器状态
/// \param ArmSocket socket句柄
/// \param voltage 返回的电压
/// \param current 返回的电流
/// \param temperature 返回的温度
/// \param sys_err 控制器运行错误代码
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Controller_State(SOCKHANDLE ArmSocket, float *voltage, float *current, float *temperature, uint16_t *sys_err);

///
/// \brief Set_WiFi_AP_Data 开启控制器WiFi AP模式设置
/// \param ArmSocket socket句柄
/// \param wifi_name 控制器wifi名称
/// \param password wifi密码
/// \return 返回值：0-成功，失败返回:错误码, rm_define.h查询.
/// 非阻塞模式，下发后，机械臂进入WIFI AP通讯模式
RM_BASESHARED_EXPORT int Set_WiFi_AP_Data(SOCKHANDLE ArmSocket, const char *wifi_name, const char* password);

///
/// \brief Set_WiFI_STA_Data 控制器WiFi STA模式设置
/// \param ArmSocket socket句柄
/// \param router_name 路由器名称
/// \param password 路由器Wifi密码
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 非阻塞模式：设置成功后，机械臂进入WIFI STA通信模式
RM_BASESHARED_EXPORT int Set_WiFI_STA_Data(SOCKHANDLE ArmSocket, const char *router_name,const char* password);

///
/// \brief Set_USB_Data 控制器UART_USB接口波特率设置
/// \param ArmSocket socket句柄
/// \param baudrate 波特率：9600,38400,115200和460800，若用户设置其他数据，控制器会默认按照460800处理。
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 非阻塞模式，设置成功后机械臂通过UART-USB接口对外通信
RM_BASESHARED_EXPORT int Set_USB_Data(SOCKHANDLE ArmSocket, int baudrate);

///
/// \brief Set_RS485 控制器RS485接口波特率设置
/// \param ArmSocket socket句柄
/// \param baudrate 波特率：9600,38400,115200和460800，若用户设置其他数据，控制器会默认按照460800处理。
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 非阻塞模式，设置成功后机械臂通过UART-USB接口对外通信
RM_BASESHARED_EXPORT int Set_RS485(SOCKHANDLE ArmSocket, int baudrate);

///
/// \brief Set_Ethernet 切换到以太网口工作模式
/// \param ArmSocket socket句柄
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
/// 非阻塞模式，设置成功后机械臂通过Ethernet接口对外通信
RM_BASESHARED_EXPORT int Set_Ethernet(SOCKHANDLE ArmSocket);

///
/// \brief Set_Arm_Power 设置机械臂电源
/// \param ArmSocket socket句柄
/// \param cmd true-上电，   false-断电
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Arm_Power(SOCKHANDLE ArmSocket, bool cmd, bool block);

///
/// \brief Get_Arm_Power_State      读取机械臂电源状态
/// \param ArmSocket                socket句柄
/// \param power                    获取到的机械臂电源状态
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                         0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Power_State(SOCKHANDLE ArmSocket, int* power);

///
/// \brief Get_Arm_Software_Version     读取软件版本号
/// \param ArmSocket                    socket句柄
/// \param plan_version                 读取到的用户接口内核版本号
/// \param ctrl_version                 实时内核版本号
/// \param kernal1                      实时内核子核心1版本号
/// \param kernal2                      实时内核子核心2版本号
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Arm_Software_Version(SOCKHANDLE ArmSocket, char* plan_version, char* ctrl_version, char *kernal1, char *kernal2);

///
/// \brief Get_System_Runtime           读取控制器的累计运行时间
/// \param ArmSocket socket句柄
/// \param state                        读取结果
/// \param day                          读取到的时间
/// \param hour                         读取到的时间
/// \param min                          读取到的时间
/// \param sec                          读取到的时间
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_System_Runtime(SOCKHANDLE ArmSocket, char* state,int* day,int* hour,int* min,int* sec);

///
/// \brief clear_system_runtime         清零控制器的累计运行时间
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Clear_System_Runtime(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Get_Joint_Odom               读取关节的累计转动角度
/// \param ArmSocket socket句柄
/// \param state                        读取结果
/// \param odom                         读取到的角度值
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Joint_Odom(SOCKHANDLE ArmSocket, char* state,float* odom);

///
/// \brief Clear_Joint_Odom             清零关节的累计转动角度
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Clear_Joint_Odom(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Set_High_Speed_Eth 基础系列设置高速网口开关
/// \param ArmSocket socket句柄
/// \param num      0-关闭  1-开启
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
RM_BASESHARED_EXPORT int Set_High_Speed_Eth(SOCKHANDLE ArmSocket, byte num, bool block);

///
/// \brief Set_IO_State 设置数字IO状态
/// \param ArmSocket socket句柄
/// \param IO 0-数字IO  1-模拟IO
/// \param num 通道号，1~4
/// \param state true-高，   false-低
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_IO_State(SOCKHANDLE ArmSocket, int IO,byte num, bool state, bool block);

///
/// \brief Get_IO_State 获取IO状态
/// \param ArmSocket socket句柄
/// \param IO       0-查询数字IO输出状态 1-查询数字IO输入状态 2-查询模拟IO输出状态 3-查询模拟IO输入状态
/// \param num      通道号，1~4
/// \param state    state 输出参数 true-高，   false-低
/// \return
///
RM_BASESHARED_EXPORT int Get_IO_State(SOCKHANDLE ArmSocket, int IO,byte num, byte *state);

///
/// \brief Get_IO_Input 查询所有数字和模拟IO的输入状态
/// \param ArmSocket socket句柄
/// \param DI_state 数字IO输入通道1~3状态数组地址，1-高，0-低
/// \param AI_voltage 模拟IO输入通道1~4输入电压数组
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_IO_Input(SOCKHANDLE ArmSocket, byte *DI_state, float *AI_voltage);

///
/// \brief Get_IO_Output 查询所有数字和模拟IO的输出状态
/// \param ArmSocket socket句柄
/// \param DO_state 数字IO输出通道1~4状态数组地址，1-高，0-低
/// \param AO_voltage 模拟IO输出通道1~4输处电压数组
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_IO_Output(SOCKHANDLE ArmSocket, byte *DO_state, float *AO_voltage);

///
/// \brief Set_Tool_DO_State 设置工具端数字IO输出
/// \param ArmSocket socket句柄
/// \param num 通道号，1~2
/// \param state true-高，   false-低
/// \paramblock RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Tool_DO_State(SOCKHANDLE ArmSocket, byte num, bool state, bool block);

///
/// \brief Set_Tool_IO_Mode 设置数字IO模式输入
/// \param ArmSocket socket句柄
/// \param num 通道号，1~2
/// \param state 0输入，   1输出
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Tool_IO_Mode(SOCKHANDLE ArmSocket, byte num, bool state, bool block);

///
/// \brief Get_Tool_IO_State 获取数字IO状态
/// \param ArmSocket socket句柄
/// \param IO_Mode 0-输入模式，1-输出模式
/// \param IO_State 0-低，1-高
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Tool_IO_State(SOCKHANDLE ArmSocket, float* IO_Mode, float* IO_State);

///
/// \brief Set_Tool_Voltage 设置工具端电压输出
/// \param ArmSocket socket句柄
/// \param type 电源输出类型，0-0V，1-5V，2-12V，3-24V
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Tool_Voltage(SOCKHANDLE ArmSocket, byte type, bool block);

///
/// \brief Get_Tool_Voltage 查询工具端电压输出
/// \param ArmSocket socket句柄
/// \param voltage 电源输出类型，0-0V，1-5V，2-12V，3-24V
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Tool_Voltage(SOCKHANDLE ArmSocket, byte *voltage);

///
/// \brief Set_Gripper_Route 设置手爪行程
/// \param ArmSocket socket句柄
/// \param min_limit 手爪最小开口，范围 ：0~1000，无单位量纲 无
/// \param max_limit 手爪最大开口，范围 ：0~1000，无单位量纲 无
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Gripper_Route(SOCKHANDLE ArmSocket, int min_limit, int max_limit, bool block);

///
/// \brief Set_Gripper_Release 手爪松开
/// \param ArmSocket socket句柄
/// \param speed 手爪松开速度 ，范围 1~1000，无单位量纲
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Gripper_Release(SOCKHANDLE ArmSocket, int speed, bool block);

///
/// \brief Set_Gripper_Pick 手爪力控夹取
/// \param ArmSocket socket句柄
/// \param speed 手爪夹取速度 ，范围 1~1000，无单位量纲 无
/// \param force 力控阈值 ，范围 ：50~1000，无单位量纲 无
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Gripper_Pick(SOCKHANDLE ArmSocket, int speed, int force, bool block);

///
/// \brief Set_Gripper_Pick_On 手爪力控持续夹取
/// \param ArmSocket socket句柄
/// \param speed 手爪夹取速度 ，范围 1~1000，无单位量纲 无
/// \param force 力控阈值 ，范围 ：50~1000，无单位量纲 无
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Gripper_Pick_On(SOCKHANDLE ArmSocket, int speed, int force, bool block);

///
/// \brief Set_Gripper_Position 设置手爪开口度
/// \param ArmSocket socket句柄
/// \param position 手爪开口位置 ，范围 ：1~1000，无单位量纲 无
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Gripper_Position(SOCKHANDLE ArmSocket, int position, bool block);

///
/// \brief Start_Drag_Teach 开始控制机械臂进入拖动示教模式
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Start_Drag_Teach(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Stop_Drag_Teach 控制机械臂退出拖动示教模式
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Stop_Drag_Teach(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Run_Drag_Trajectory 控制机械臂复现拖动示教的轨迹，必须在拖动示教结束后才能使用，
///                            同时保证机械臂位于拖动示教的起点位置。
///                            若当前位置没有位于轨迹复现起点，请先调用Drag_Trajectory_Origin，否则会返回报错信息。
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Run_Drag_Trajectory(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Pause_Drag_Trajectory 控制机械臂在轨迹复现过程中的暂停
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Pause_Drag_Trajectory(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Continue_Drag_Trajectory 控制机械臂在轨迹复现过程中暂停之后的继续，
///                                 轨迹继续时，必须保证机械臂位于暂停时的位置，
///                                 否则会报错，用户只能从开始位置重新复现轨迹。
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Continue_Drag_Trajectory(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Stop_Drag_Trajectory 控制机械臂在轨迹复现过程中的停止
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Stop_Drag_Trajectory(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Drag_Trajectory_Origin 轨迹复现前，必须控制机械臂运动到轨迹起点，
///                               如果设置正确，机械臂将以20%的速度运动到轨迹起点
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Drag_Trajectory_Origin(SOCKHANDLE ArmSocket, int block);

///
/// \brief Start_Multi_Drag_Teach       开始复合模式拖动示教
/// \param ArmSocket socket句柄
/// \param mode                         拖动示教模式 0-电流环模式，1-使用末端六维力，只动位置，2-使用末端六维力 ，只动姿态，3-使用末端六维力，位置和姿态同时动
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Start_Multi_Drag_Teach(SOCKHANDLE ArmSocket, int mode,bool block);

///
/// \brief Set_Force_Postion            力位混合控制
/// \param ArmSocket                    socket句柄
/// \param sensor                       0-一维力；1-六维力
/// \param mode                         0-基坐标系力控；1-工具坐标系力控；
/// \param direction                    力控方向；0-沿X轴；1-沿Y轴；2-沿Z轴；3-沿RX姿态方向；4-沿RY姿态方向；5-沿RZ姿态方向
/// \param N                            力的大小，单位N
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Force_Postion(SOCKHANDLE ArmSocket, int sensor, int mode, int direction, int N, int block);

///
/// \brief Stop_Force_Postion           结束力位混合控制
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Stop_Force_Postion(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Get_Force_Data 查询当前六维力传感器得到的力和力矩信息，若要周期获取力数据
///                       周期不能小于50ms。
/// \param ArmSocket socket句柄
/// \param Force 返回的力和力矩数组地址，数组6个元素，依次为Fx，Fy，Fz, Mx, My, Mz。
///              其中，力的单位为N；力矩单位为Nm。
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Force_Data(SOCKHANDLE ArmSocket, float *Force);


///
/// \brief Set_Force_Sensor 设置六维力重心参数，六维力重新安装后，必须重新计算六维力所收到的初始力和重心。
///                         该指令下发后，机械臂以20%的速度运动到各标定点，该过程不可中断，中断后必须重新标定。
///                         重要说明：必须保证在机械臂静止状态下标定。
/// \param ArmSocket socket句柄
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Force_Sensor(SOCKHANDLE ArmSocket);

///
/// \brief Manual_Set_Force 手动设置六维力重心参数，六维力重新安装后，必须重新计算六维力所收到的初始力和重心。
/// \param ArmSocket socket句柄
/// \param type 点位；1~4，调用此函数四次
/// \param joint 关节角度
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Manual_Set_Force(SOCKHANDLE ArmSocket, int type,const float* joint);

///
/// \brief Stop_Set_Force_Sensor 在标定六维力过程中，如果发生意外，发送该指令，停止机械臂运动，退出标定流程
/// \param ArmSocket socket句柄
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Stop_Set_Force_Sensor(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Set_Hand_Posture 设置灵巧手目标手势
/// \param ArmSocket socket句柄
/// \param posture_num 预先保存在灵巧手内的手势序号，范围：1~40
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回; RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Hand_Posture(SOCKHANDLE ArmSocket, int posture_num, bool block);

///
/// \brief Set_Hand_Seq 设置灵巧手目标动作序列
/// \param ArmSocket socket句柄
/// \param seq_num 预先保存在灵巧手内的动作序列序号，范围：1~40
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Hand_Seq(SOCKHANDLE ArmSocket, int seq_num, bool block);

///
/// \brief Set_Hand_Angle 设置灵巧手各关节角度
/// \param ArmSocket socket句柄
/// \param angle 手指角度数组，6个元素分别代表6个自由度的角度。范围：0~1000.另外，-1代表该自由度不执行任何操作，保持当前状态
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Hand_Angle(SOCKHANDLE ArmSocket, const int *angle, bool block);

///
/// \brief Set_Hand_Speed 设置灵巧手各关节速度
/// \param ArmSocket socket句柄
/// \param speed 灵巧手各关节速度设置，范围：1~1000
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Hand_Speed(SOCKHANDLE ArmSocket, int speed, bool block);

///
/// \brief Set_Hand_Force 设置灵巧手各关节力阈值
/// \param ArmSocket socket句柄
/// \param force 灵巧手各关节力阈值设置，范围：1~1000，代表各关节的力矩阈值（四指握力0~10N，拇指握力0~15N）。
/// \param block RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Hand_Force(SOCKHANDLE ArmSocket, int force, bool block);

///
/// \brief Get_Fz 该函数用于查询末端一维力数据
/// \param ArmSocket socket句柄
/// \param 反馈的一维力数据
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Fz(SOCKHANDLE ArmSocket, float *Fz);

///
/// \brief Clear_Fz 该函数用于清零末端一维力数据
/// \param ArmSocket socket句柄
/// \param RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Clear_Fz(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Auto_Set_Fz 该函数用于自动一维力数据
/// \param ArmSocket socket句柄
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Auto_Set_Fz(SOCKHANDLE ArmSocket);

///
/// \brief Manual_Set_Fz 该函数用于手动设置一维力数据
/// \param ArmSocket socket句柄
/// \param 关节角度
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Manual_Set_Fz(SOCKHANDLE ArmSocket, const float* joint1,const float* joint2);

///
/// \brief 配置通讯端口 Modbus RTU 模式
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param baudrate: 波特率，支持 9600,115200,460800 三种常见波特率
/// \param timeout: 超时时间，单位秒。
/// \param block: RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Modbus_Mode(SOCKHANDLE ArmSocket, int port,int baudrate,int timeout,bool block);

///
/// \brief 关闭通讯端口 Modbus RTU 模式
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param block: RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Close_Modbus_Mode(SOCKHANDLE ArmSocket, int port, bool block);

///
/// \brief 读线圈
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param address: 线圈起始地址
/// \param num: 要读的线圈的数量，该指令最多一次性支持读 8 个线圈数据，即返回的数据不会一个字节
/// \param device: 外设设备地址
/// \param coils_data: 返回离散量
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Read_Coils(SOCKHANDLE ArmSocket, int port,int address,int num,int device,int *coils_data);

///
/// \brief Get_Read_Input_Status 读离散量输入
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param address: 线圈起始地址
/// \param num: 要读的线圈的数量，该指令最多一次性支持读 8 个线圈数据，即返回的数据不会一个字节
/// \param device: 外设设备地址
/// \param coils_data: 返回离散量
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Read_Input_Status(SOCKHANDLE ArmSocket, int port,int address,int num,int device,int* coils_data);

///
/// \brief Get_Read_Holding_Registers 读保持寄存器
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param address: 线圈起始地址
/// \param device: 外设设备地址
/// \param coils_data: 返回离散量
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Read_Holding_Registers(SOCKHANDLE ArmSocket, int port,int address,int device,int* coils_data);

///
/// \brief Get_Read_Input_Registers 读输入寄存器
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param address: 线圈起始地址
/// \param device: 外设设备地址
/// \param coils_data: 返回离散量
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Read_Input_Registers(SOCKHANDLE ArmSocket, int port,int address,int device,int* coils_data);

///
/// \brief Write_Single_Coil 写单圈数据
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param address: 线圈起始地址
/// \param data: 要读的线圈的数量，该指令最多一次性支持读 8 个线圈数据，即返回的数据不会一个字节
/// \param device: 外设设备地址
/// \param block: RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Write_Single_Coil(SOCKHANDLE ArmSocket, int port,int address,int data,int device,bool block);

///
/// \brief Write_Coils 写多圈数据
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器RS485端口，1-末端接口板RS485接口
/// \param address: 线圈起始地址
/// \param num: 写线圈个数，每次写的数量不超过160个
/// \param coils_data: 要写入线圈的数据数组，类型：byte。若线圈个数不大于8，则写入的数据为1个字节；否则，则为多个数据的数组
/// \param device: 外设设备地址
/// \param block: RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Write_Coils(SOCKHANDLE ArmSocket, int port,int address,int num, byte * coils_data,int device,bool block);

///
/// \brief Write_Single_Register 写单个寄存器
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param address: 线圈起始地址
/// \param data: 要读的线圈的数量，该指令最多一次性支持读 8 个线圈数据，即返回的数据不会一个字节
/// \param device: 外设设备地址
/// \param block: RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Write_Single_Register(SOCKHANDLE ArmSocket, int port,int address,int data,int device,bool block);

///
/// \brief Write_Registers 写多个寄存器
/// \param ArmSocket socket句柄
/// \param port: 通讯端口，0-控制器 RS485 端口，1-末端接口板 RS485 接口
/// \param address: 寄存器起始地址
/// \param num: 写寄存器个数，寄存器每次写的数量不超过10个
/// \param single_data: 要写入寄存器的数据数组，类型：byte
/// \param device: 外设设备地址
/// \param block: RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return 0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Write_Registers(SOCKHANDLE ArmSocket, int port,int address,int num, byte *single_data, int device, bool block);

///
/// \brief Set_Lift_Speed           速度开环控制
/// \param ArmSocket                socket句柄
/// \param speed                    speed-速度百分比，-100 ~100
/// \return                         0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Lift_Speed(SOCKHANDLE ArmSocket, int speed);

///
/// \brief Set_Lift_Height          位置闭环控制
/// \param ArmSocket                socket句柄
/// \param height                   目标高度，单位mm，范围：0~2600
/// \param speed                    速度百分比，1~100
/// \param block                    RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                         0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Lift_Height(SOCKHANDLE ArmSocket, int height,int speed,bool block);

///
/// \brief Get_Lift_State           获取升降机构状态
/// \param ArmSocket socket句柄
/// \param height                   当前升降机构高度，单位：mm，精度：1mm，范围：0~2300
/// \param current                  当前升降驱动电流，单位：mA，精度：1mA
/// \param err_flag                 升降驱动错误代码，错误代码类型参考关节错误代码
/// \param block                    RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                         0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Get_Lift_State(SOCKHANDLE ArmSocket, int* height,int* current,int* err_flag);

///
/// \brief Start_Force_Position_Move    开启透传力位混合控制补偿模式
/// \param ArmSocket                    socket句柄
/// \param block                        RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Start_Force_Position_Move(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Force_Position_Move          透传力位混合补偿
/// \param ArmSocket                    socket句柄
/// \param pose                         当前坐标系下目标位姿
/// \param joint                        目标关节角度
/// \param sensor                       所使用传感器类型，0-一维力，1-六维力
/// \param mode                         模式，0-沿基坐标系，1-沿工具端坐标系
/// \param dir                          力控方向，0~5分别代表X/Y/Z/Rx/Ry/Rz，其中一维力类型时默认方向为Z方向
/// \param force                        力的大小 单位N
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Force_Position_Move_POSE(SOCKHANDLE ArmSocket, POSE pose,byte sensor,byte mode,int dir,float force);
RM_BASESHARED_EXPORT int Force_Position_Move_Joint(SOCKHANDLE ArmSocket, const float *joint,byte sensor,byte mode,int dir,float force);

///
/// \brief Stop_Force_Postion_Move          停止透传力位混合控制补偿模式
/// \param ArmSocket                    socket句柄
/// \param block                        RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Stop_Force_Postion_Move(SOCKHANDLE ArmSocket, bool block);

///
/// \brief Set_Teach_Frame              切换示教运动坐标系
/// \param ArmSocket                    socket句柄
/// \param type                         0: 基座标运动, 1: 工具坐标系运动
/// \param block                        RM_NONBLOCK-非阻塞，发送后立即返回；RM_BLOCK-阻塞，等待控制器返回设置成功指令
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Set_Teach_Frame(SOCKHANDLE ArmSocket, int type, int block);

///
/// \brief Stop_Force_Postion_Move      轨迹文件下发
/// \param ArmSocket                    socket句柄
/// \param file_name                    轨迹文件完整路径 例: c:/rm_file.txt
/// \param file_name_len                file_name 字段的长度
/// \param plan_speed                   规划速度比例(0-100)
/// \return                             0-成功，失败返回:错误码, rm_define.h查询.
///
RM_BASESHARED_EXPORT int Send_TrajectoryFile(SOCKHANDLE ArmSocket, char * file_name, int file_name_len, int plan_speed);
// ************************************* 算法封装 *********************

///
/// \brief Init_Sys_Data  初始化算法依赖数据(不连接机械臂时调用, 连接机械臂会自动调用)
/// \param dMode   机械臂型号ARM_65 ARM_75 ARM_63_1 ARM_63_2
///
RM_BASESHARED_EXPORT void Init_Sys_Data(int dMode);
///
/// \brief setAngle  设置安装角度
/// \param x   X轴安装角度 单位°
/// \param y   Y轴安装角度 单位°
/// \param z   z轴安装角度 单位°
///
RM_BASESHARED_EXPORT void setAngle(float x,float y, float z);

///
/// \brief setLwt       设置末端DH参数
/// \param type         0-标准版，1-一维力，2-六维力
///
RM_BASESHARED_EXPORT void setLwt(int type);

///
/// \brief Forward_Kinematics  正解函数
/// \param joint               关节1到关节6角度 单位°
/// \return
///
RM_BASESHARED_EXPORT Pose Forward_Kinematics(const float * joint);

///
/// \brief inverse_kinematics  逆解函数
/// \param q_in                上一时刻关节角度 单位°
/// \param q_pose              目标位姿
/// \param q_out               输出的关节角度 单位°
/// \param flag                姿态参数类别：0-四元数；1-欧拉角
/// \return                    SYS_NORMAL：计算正常，CALCULATION_FAILED：计算失败
///
RM_BASESHARED_EXPORT int inverse_kinematics(const float *q_in, const Pose *q_pose, float *q_out, const uint8_t flag);

///
/// \brief RotateMove                   计算环绕运动位姿
/// \param Joint_Cur                    当前关节角度 单位°
/// \param rotateAxis                   旋转轴: 1:x轴, 2:y轴, 3:z轴
/// \param rotateAngle                  旋转角度: 旋转角度, 单位(度)
/// \param choose_axis                  指定计算时使用的坐标系
/// \return                             计算位姿结果
RM_BASESHARED_EXPORT Pose RotateMove(float *Joint_Cur, int rotateAxis, float rotateAngle, Pose choose_axis);

///
/// \brief end2tool             末端位姿转成工具位姿
/// \param eu_end               基于世界坐标系和默认工具坐标系的末端位姿
/// \param co_tool              工具坐标系位姿
/// \param co_work              工作坐标系位姿
/// \return                     基于工作坐标系和工具坐标系的末端位姿
///
RM_BASESHARED_EXPORT Pose end2tool(Pose eu_end, Pose co_tool, Pose co_work);

///
/// \brief tool2end             工具位姿转末端位姿
/// \param eu_tool              基于工作坐标系和工具坐标系的末端位姿
/// \param co_tool              工具坐标系位姿
/// \param co_work              工作坐标系位姿
/// \return                     基于世界坐标系和默认工具坐标系的末端位姿
///
RM_BASESHARED_EXPORT Pose tool2end(Pose eu_tool, Pose co_tool, Pose co_work);

///
/// \brief quaternion2euler  四元数转欧拉角
/// \param qua               四元数
/// \return                  欧拉角
///
RM_BASESHARED_EXPORT eul quaternion2euler(ort qua);

///
/// \brief quaternion2euler  欧拉角转四元数
/// \param eu                欧拉角
/// \return                  四元数
///
RM_BASESHARED_EXPORT ort euler2quaternion(eul eu);

///
/// \brief euler2matrix      欧拉角转旋转矩阵
/// \param eu                欧拉角
/// \return                  旋转矩阵
///
RM_BASESHARED_EXPORT Matrix euler2matrix(eul eu);

///
/// \brief pos2matrix      位姿转旋转矩阵
/// \param pos             位姿
/// \return                旋转矩阵
///
RM_BASESHARED_EXPORT Matrix pos2matrix(Pose pos);

///
/// \brief matrix2pos      旋转矩阵转位姿
/// \param Mstate          旋转矩阵
/// \return                位姿
///
RM_BASESHARED_EXPORT Pose matrix2pos(Matrix Mstate);

///
/// \brief Base_To_WorkFrame      基座标系转工作坐标系
/// \param state                  工具端坐标在基座标系下位姿
/// \param matrix                 工作坐标系在基座标系下的矩阵
/// \return                       基座标系在工作坐标系下的矩阵
///
RM_BASESHARED_EXPORT Pose Base_To_WorkFrame(Matrix matrix, Pose state);

///
/// \brief WorkFrame_To_Base      工作坐标系转基座标系
/// \param state                  工具端坐标在工作座标系下位姿
/// \param matrix                 工作坐标系在基座标系下的矩阵
/// \return                       工作坐标系下的位姿
///
RM_BASESHARED_EXPORT Pose WorkFrame_To_Base(Matrix matrix, Pose state);

///
/// \brief Cartesian_Tool           计算沿工具坐标系运动位姿
/// \param Joint_Cur                当前关节角度
/// \param movelengthx:             沿X轴移动长度，米为单位
/// \param movelengthy:             沿Y轴移动长度，米为单位
/// \param movelengthz:             沿Z轴移动长度，米为单位
/// \param m_dev                    机械臂型号
/// \return                         工作坐标系下的位姿
///
RM_BASESHARED_EXPORT Pose Cartesian_Tool(float *Joint_Cur, float movelengthx, float movelengthy, float movelengthz, int dev_mode);
#ifdef __cplusplus
}
#endif
#endif // RM_BASE_H
