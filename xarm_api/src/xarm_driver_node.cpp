/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: waylon <weile.wang@ufactory.cc>
 ============================================================================*/
#include <xarm_driver.h>
#include <xarm/linux/thread.h>
#include "xarm/connect.h"
#include "xarm/report_data.h"

class XarmRTConnection
{
    public:
        XarmRTConnection(ros::NodeHandle& root_nh, char *server_ip, xarm_api::XARMDriver &drv)
        {   
            root_nh.getParam("DOF", joint_num_);
            root_nh.getParam("joint_names", joint_name_);
            ip = server_ip;
            xarm_driver = drv;
            xarm_driver.XARMDriverInit(root_nh, server_ip);
            thread_id = thread_init(thread_proc, (void *)this);

        }

        void thread_run(void)
        {
            int ret;
            int err_num;
            int rxcnt;
            int i;

            ros::Rate r(50); // 50Hz

            while(true)
            {
                // usleep(5000);
                ret = xarm_driver.get_frame();
                if (ret != 0) continue;

                ret = xarm_driver.get_rich_data(norm_data);
                if (ret == 0)
                {
                    rxcnt++;

                    now = ros::Time::now();
                    js_msg.header.stamp = now;
                    js_msg.header.frame_id = "real-time data";
                    js_msg.name.resize(joint_num_);
                    js_msg.position.resize(joint_num_);
                    js_msg.velocity.resize(joint_num_);
                    js_msg.effort.resize(joint_num_);
                    for(i = 0; i < joint_num_; i++)
                    {
                        double d;
                        d = (double)norm_data.angle_[i];
                        js_msg.name[i] = joint_name_[i];
                        js_msg.position[i] = d;
                        js_msg.velocity[i] = 0;
                        js_msg.velocity[i] = 0;
                    }
                    
                    xarm_driver.pub_joint_state(js_msg);

                    rm_msg.state = norm_data.runing_;
                    rm_msg.mode = norm_data.mode_;
                    rm_msg.cmdnum = norm_data.cmdnum_;
                    rm_msg.err = norm_data.err_;
                    rm_msg.warn = norm_data.war_;
                    rm_msg.mt_brake = norm_data.mt_brake_;
                    rm_msg.mt_able = norm_data.mt_able_;
                    rm_msg.angle.resize(joint_num_);
                    
                    for(i = 0; i < joint_num_; i++)    
                    {
                        /* set the float precision*/
                        double d = norm_data.angle_[i];
                        double r;
                        char str[8];
                        sprintf(str, "%0.3f", d); 
                        sscanf(str, "%lf", &r);
                        rm_msg.angle[i] = r;
                    }
                    for(i = 0; i < 6; i++)    
                    {
                        rm_msg.pose[i] = norm_data.pose_[i];
                        rm_msg.offset[i] = norm_data.tcp_offset_[i];
                    }
                    xarm_driver.pub_robot_msg(rm_msg);
                }
                else 
                {
                    printf("Error: real_data.flush_data failed, ret = %d\n", ret);
                    err_num++;
                }

                r.sleep();
            }
        }

        static void* thread_proc(void *arg) 
        {
            XarmRTConnection* pThreadTest=(XarmRTConnection*)arg;
            pThreadTest->thread_run();
        }

    public:
        pthread_t thread_id;
        char *ip;
        ros::Time now;
        SocketPort *arm_report;
        ReportDataNorm norm_data;
        sensor_msgs::JointState js_msg;
        xarm_api::XARMDriver xarm_driver;
        xarm_msgs::RobotMsg rm_msg;

        int joint_num_;
        std::vector<std::string> joint_name_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xarm_driver_node");
    
    ros::NodeHandle n;

    xarm_api::XARMDriver driver;
    ROS_INFO("start xarm driver");

    std::string robot_ip = "192.168.1.121";
    if (!n.hasParam("xarm_robot_ip"))
    {
        ROS_INFO("No param named 'xarm_robot_ip'");
        ROS_INFO("Use default robot ip = 192.168.1.121");
    }
    else
    {
        n.getParam("xarm_robot_ip", robot_ip);
    }

    char server_ip[20];
    strcpy(server_ip,robot_ip.c_str());
    XarmRTConnection rt_connect(n, server_ip, driver);

    // ros::spin();
    ros::waitForShutdown();

    printf("end");
    
    return 0;
}
