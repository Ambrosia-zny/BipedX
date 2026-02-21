#include <stdio.h>

typedef struct _RobotState{
    double angle;
    double speed;
} RobotState;

void modifyRobotState(RobotState *robot, double newAngle, double newSpeed);
void printRobotState(RobotState *robot);

int main(){
    RobotState robot = {90.0, 0.0}; // 初始数据
    printf("初始状态：\n");
    printRobotState(&robot); // 打印初始状态
    
    while(1){
        double angle = 90.0, speed = 0.0; // 状态量
        
        printf("\n请输入新的角度和速度（空格分隔）：");
        scanf("%lf %lf", &angle, &speed);
        
        if(angle < 0.0 || angle > 180.0 || speed < 0.0){
            printf("程序退出\n");//输入角度小于0或大于180，或速度小于0时退出循环
            break;
        } else {
            modifyRobotState(&robot, angle, speed); // 修改状态
            printf("当前状态：\n");
            printRobotState(&robot); // 打印当前状态
        }
    }
    
    return 0;
}

void modifyRobotState(RobotState *robot, double newAngle, double newSpeed){
    if(robot != NULL){
        robot->angle = newAngle;
        robot->speed = newSpeed;
        printf("状态更新完毕\n");
    } else {
        printf("错误：空指针\n");
    }
}

void printRobotState(RobotState *robot){
    if(robot != NULL){
        printf("角度：%.2f \n", robot->angle);
        printf("速度：%.2f \n", robot->speed);
    } else {
        printf("错误：空指针\n");
    }
}