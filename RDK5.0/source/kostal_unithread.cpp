/**
 * @example kostal.cpp
 * Select a plan from a list to execute using RDK's plan execution API.
 * @copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <Robot.hpp>
#include <Exception.hpp>
#include <Log.hpp>
#include <Scheduler.hpp>
#include <string>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include "ControlSPI.h"
#include <stdio.h>
#include <unistd.h>
#include <stack>
#include <array>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <math.h>
#include "math.h"
#include <eigen3/Eigen/Eigen>
#include <atomic>
using namespace std;

double robotData[20] ={0};
uint8_t spiData[16] = {0};
std::array<uint8_t, 16> SPI_unit;
int ret;
bool stackFlag = false;
std::atomic<bool> runSwitch(true);
std::atomic<bool> runSwitch2(true);
double M_Pi;
double rad = 180/M_Pi;
Eigen::Quaternion<double> q; // new a quaternion

//stack for data storage
std::stack<std::array<double, 16>> robot_stack;
std::stack<std::array<double, 16>> robot_stack_rev;

std::stack<std::array<uint8_t, 16>> SPI_stack;
std::stack<std::array<uint8_t, 16>> SPI_stack_rev;

std::stack<std::string> nodeName_stack;
std::stack<std::string> nodeName_stack_rev;

struct PrintData
{
    std::vector<double> tcp_pose;
    std::vector<double> raw_sensor_data;
    std::vector<double> flangePose;
    std::string nodeName;

} g_printData;

struct Euler_out
{
    double Roll,Pitch,Yaw;

}g_euler_Out,g_euler_Out_f;

std::mutex g_printDataMutex;
std::mutex SPImutex;

//test function uint8 to binary 
void toBinary(uint8_t a)
{
    uint8_t i;

    for(i=0x80;i!=0;i>>=1)
        printf("%c",(a&i)?'1':'0'); 
}

//Quaternion to Euler

typedef struct Euler_out Struct;

Struct Quaternion_to_Euler (double w, double x, double y, double z)
{
    q.w() = w;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);
    
    
    Struct Eout;
    {
        Eout.Roll = euler[0];
        Eout.Pitch = euler[1];
        Eout.Yaw = euler[2];
    }
    return Eout;
}

void customizedTriggerTask(flexiv::Robot* robot,
                           flexiv::RobotStates* robotStates,
                           //std::string* nodeName)
                           flexiv::PlanInfo* planInfo)
{
    robot->getRobotStates(robotStates);
    robot->getPlanInfo(planInfo);
    g_printData.tcp_pose = robotStates->m_tcpPose;
    g_printData.raw_sensor_data = robotStates->m_rawExtForceInTcpFrame;
    g_printData.flangePose = robotStates->m_flangePose;
    g_printData.nodeName = planInfo->m_nodeName;
    //g_printData.nodeName = *nodeName;
    for (int i=0; i<7; i++){
        robotData[i] = g_printData.tcp_pose[i];
    }
    for (int i=0; i<6; i++){
        robotData[i+7] = g_printData.raw_sensor_data[i];
    }
    for (int i=0; i<3; i++){
        robotData[i+13] = g_printData.flangePose[i];
    }
    robot_stack.push({robotData[0],robotData[1],robotData[2],robotData[3],robotData[4],robotData[5],robotData[6],robotData[7],robotData[8],robotData[9],robotData[10],robotData[11],robotData[12],robotData[13],robotData[14],robotData[15]/*,robotData[16],robotData[17],robotData[18],robotData[19]*/});
    nodeName_stack.push(g_printData.nodeName);
}

//Read SPIdata
void SPIdata_collection(flexiv::Robot* robot,
                        flexiv::RobotStates* robotStates,
                        flexiv::PlanInfo* planInfo)
{
    while (runSwitch2)
    {
        uint8_t read_buffer[10240] = {0};
        int32_t read_data_num = 0;
        ret = VSI_SlaveReadBytes(VSI_USBSPI, 0, read_buffer, &read_data_num,100);
        if (ret != ERR_SUCCESS)
        {
            printf("Read data error!\n");
            continue;
        }else
        {
            flexiv::SystemStatus systemStatus;
            robot->getSystemStatus(&systemStatus);
            do {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                robot->getSystemStatus(&systemStatus);

            } while (systemStatus.m_programRunning != true);
            
            robot->getPlanInfo(planInfo);
            
            std::string nodename = planInfo->m_nodeName;
            
            if (nodename == "Start")
            {
                stackFlag = true;
            }
            if (nodename == "Stop")
            {
                stackFlag = false;
            }
            if (read_data_num == 16 && stackFlag == true)
            {
                std::lock_guard<std::mutex> lock(SPImutex);
                for (int i = 0; i < read_data_num; i++){
                    spiData[i]=read_buffer[i];
                    //SPI_unit[i]=read_buffer[i];
                }
                //SPI_stack.push(SPI_unit);
                //SPI_stack.push({spiData[0],spiData[1],spiData[2],spiData[3],spiData[4],spiData[5],spiData[6],spiData[7],spiData[8],spiData[9],spiData[10],spiData[11],spiData[12],spiData[13],spiData[14],spiData[15]});
                //customizedTriggerTask(robot, robotStates, planInfo);
                //customizedTriggerTask(robot, robotStates, &nodename); // direct use nodename found just now
            }
            if (nodename == "End0" || nodename == "End01"){
                runSwitch = false;
            }
        }   
    }
}

int sendRobotPlan(flexiv::Robot* robot,
                  flexiv::RobotStates* robotStates,
                  flexiv::PlanInfo* planInfo, 
                  std::string planName,
                  std::string fileName,
                  std::string filePath)
{

    std::thread SPIthread(std::bind(SPIdata_collection,robot,robotStates,planInfo));
    std::thread Robthread();
    robot->executePlanByName(planName);               
    flexiv::SystemStatus systemStatus;
    robot->getSystemStatus(&systemStatus);

    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        robot->getSystemStatus(&systemStatus);

    } while (systemStatus.m_programRunning != true);

    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        robot->getSystemStatus(&systemStatus);
    } while (systemStatus.m_programRunning == true);
    std::cout<<"The task is finished"<<std::endl;
    SPIthread.join();
    std::cout<<"The spi thread is finished"<<std::endl;
    std::cout<<"The runSwitch is "<< runSwitch<<std::endl;
    std::cout<<"The collect data number is: "<< SPI_stack.size()<<std::endl;

    while (!robot_stack.empty())            
    {
        robot_stack_rev.push({robot_stack.top()[0],robot_stack.top()[1],robot_stack.top()[2],robot_stack.top()[3],robot_stack.top()[4],robot_stack.top()[5],robot_stack.top()[6],robot_stack.top()[7],robot_stack.top()[8],robot_stack.top()[9],robot_stack.top()[10],robot_stack.top()[11],robot_stack.top()[12],robot_stack.top()[13],robot_stack.top()[14],robot_stack.top()[15]/*,robot_stack.top()[16],robot_stack.top()[17],robot_stack.top()[18],robot_stack.top()[19]*/});
        SPI_stack_rev.push({SPI_stack.top()[0],SPI_stack.top()[1],SPI_stack.top()[2],SPI_stack.top()[3],SPI_stack.top()[4],SPI_stack.top()[5],SPI_stack.top()[6],SPI_stack.top()[7],SPI_stack.top()[8],SPI_stack.top()[9],SPI_stack.top()[10],SPI_stack.top()[11],SPI_stack.top()[12],SPI_stack.top()[13],SPI_stack.top()[14],SPI_stack.top()[15]});
        nodeName_stack_rev.push(nodeName_stack.top());
        
        robot_stack.pop();
        SPI_stack.pop();
        nodeName_stack.pop();
    }
    
    std::cout<<"generating the file..."<<std::endl;
    std::time_t file_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string time_str = std::ctime(&file_time); 

    std::ofstream MyExcelFile;
    std::string tmpFileName= filePath+fileName+".csv";
    MyExcelFile.open(tmpFileName);
    std::cout<< tmpFileName<<std::endl;
    
    //add file headers  
    MyExcelFile << "NodeName"<<",";
    MyExcelFile << "TCP_x"<<","<<"TCP_y"<<"," << "TCP_z"<<"," ;
    MyExcelFile << "TCP_Rx"<<","<<"TCP_Ry"<<"," << "TCP_Rz"<<"," ;
    MyExcelFile << "FLANGE_x"<<","<<"FLANGE_y"<<"," << "FLANGE_z"<<"," ;
    // MyExcelFile << "FLANGE_Rx"<<","<<"FLANGE_Ry"<<"," << "FLANGR_Rz"<<"," ;
    MyExcelFile << "RowDataSensor0"<<","<< "RowDataSensor1"<<","<< "RowDataSensor2"<<",";
    MyExcelFile << "RowDataSensor3"<<","<< "RowDataSensor4"<<","<< "RowDataSensor5"<<",";
    MyExcelFile << "SPI0-0"<<","<< "SPI0-1"<<","<< "SPI0-2"<<","<< "SPI0-3"<<","<< "SPI0-4"<<",";
    MyExcelFile << "SPI0-5"<<","<< "SPI0-6"<<","<< "SPI0-7"<<",";
    MyExcelFile << "SPI1-0"<<","<< "SPI1-1"<<","<< "SPI1-2"<<","<< "SPI1-3"<<","<< "SPI1-4"<<",";
    MyExcelFile << "SPI1-5"<<","<< "SPI1-6"<<","<< "SPI1-7"<<",";
    MyExcelFile << std::endl; 

    std::cout<< ' ' <<robot_stack_rev.size() << std::endl;
    std::cout<< ' ' <<SPI_stack_rev.size() << std::endl;

    while (!robot_stack_rev.empty())            
    {
        MyExcelFile << nodeName_stack_rev.top()<<",";
        auto showdata = robot_stack_rev.top();           
        uint8_t showspi = SPI_stack_rev.top()[6];           
    
        g_euler_Out = Quaternion_to_Euler(robot_stack_rev.top()[3],robot_stack_rev.top()[4],robot_stack_rev.top()[5],robot_stack_rev.top()[6]);
        // g_euler_Out_f = Quaternion_to_Euler(robot_stack_rev.top()[16],robot_stack_rev.top()[17],robot_stack_rev.top()[18],robot_stack_rev.top()[19]);

        //
        MyExcelFile << robot_stack_rev.top()[0]<<",";
        MyExcelFile << robot_stack_rev.top()[1]<<",";
        MyExcelFile << robot_stack_rev.top()[2]<<",";

        MyExcelFile << g_euler_Out.Roll<<",";
        MyExcelFile << g_euler_Out.Pitch<<",";
        MyExcelFile << g_euler_Out.Yaw<<",";

        MyExcelFile << robot_stack_rev.top()[13]<<",";
        MyExcelFile << robot_stack_rev.top()[14]<<",";
        MyExcelFile << robot_stack_rev.top()[15]<<",";
        
        // MyExcelFile << g_euler_Out_f.Roll<<",";
        // MyExcelFile << g_euler_Out_f.Pitch<<",";
        // MyExcelFile << g_euler_Out_f.Yaw<<",";
        //write sensor data
        for (auto i = 7; i < 13; i++){
        MyExcelFile << robot_stack_rev.top()[i]<<",";
        }
        
        //Write SPIdata
        for (auto i = 0; i < 16; i++){
        MyExcelFile << std::setfill('0') << std::setw(2) << std::right<<std::hex ;
        MyExcelFile << + static_cast<uint8_t>(SPI_stack_rev.top()[i])<<",";
        }
        MyExcelFile << std::endl; 


        //check syncsats
        //printf("%02X ",showspi);
        //std::cout<< ' ' <<showdata[13]<< std::endl;
        //std::cout<<nodeName_stack_rev.top()<<std::endl;
        robot_stack_rev.pop();
        SPI_stack_rev.pop();
        nodeName_stack_rev.pop();
    }

    MyExcelFile.close();
    std::cout<< tmpFileName<<std::endl;

    return 0; 
}

int main()
{
    flexiv::Log log;
    // IP of the robot server
    std::string robotIP = "192.168.2.100";
    //std::string robotIP = "127.0.0.1";
    // IP of the workstation PC running this program
    std::string localIP = "192.168.2.104";
    //std::string localIP = "127.0.0.1";
    // create data struct for storing robot states and plan info
    flexiv::RobotStates robotStates;
    flexiv::PlanInfo planInfo;

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 0;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");

        try {
            robot.enable();
        }
        catch (const flexiv::Exception& e) {
            log.error(e.what());
            return 0;
        }

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.info("Robot is now operational");

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_PLAN_EXECUTION);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_PLAN_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        log.info("Robot is now in execution mode");

        // SPI part
        VSI_INIT_CONFIG SPI_Config;
        // Scan connected device
        ret = VSI_ScanDevice(1);
        if (ret <= 0) {
            log.error("No device connect!");
            return ret;
        }
        // Open device
        ret = VSI_OpenDevice(VSI_USBSPI, 0, 0);
        if (ret != ERR_SUCCESS) {
            log.error("Open device error!");
            return ret;
        }

        // Initialize device (Slave Mode, Hardware SPI, Full-Duplex)
        SPI_Config.ControlMode = 0;
        SPI_Config.MasterMode = 0; // Slave Mode
        SPI_Config.CPHA = 1; // Clock Polarity and Phase must be same as master
        SPI_Config.CPOL = 0;
        SPI_Config.LSBFirst = 0;
        SPI_Config.TranBits = 8; // Support 8 bit mode only
        SPI_Config.SelPolarity = 0;
        SPI_Config.ClockSpeed = 1395000;
        ret = VSI_InitSPI(VSI_USBSPI, 0, &SPI_Config);
        if (ret != ERR_SUCCESS) 
        {
            log.error("Initialize device error!");
            return ret;
        }
        log.info("SPI Initialize device successfully!");

        std::time_t file_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::string time_str = std::ctime(&file_time);
        std::string planName="Kostal-MainPlan";
        std::string fileName=planName+time_str;
        std::string filePath="./data/";

        int success_flag = sendRobotPlan(&robot, &robotStates, &planInfo, planName, fileName ,filePath);
        
        //std::cout << success_flag << std::endl;
    
        return 1;
    }catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 0;
    }
}
