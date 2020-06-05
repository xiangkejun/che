#include "sineBot_base.h"
#include "modbus/modbus.h"

// Bit field and masking macros
#define bit(n) (1 << n)
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)
//请求控制字定义
#define REQ_INIT_MOTOR  bit(0)//初始化电机

//状态字定义
#define STATE_INIT_MOTOR bit(0)//电机初始化情况
#define FLOAT_SIZE 2
//modebus变量地址，状态为输入寄存器，控制为保持寄存器
#define LINEAR_VEL_X_ADDR 0
#define ANGULAR_VEL_Z_ADDR (LINEAR_VEL_X_ADDR + FLOAT_SIZE)
#define STATE_ADDR (ANGULAR_VEL_Z_ADDR + FLOAT_SIZE)

modbus_t *m_modbus;
bool init_modbus(std::string serial_port)
{
    //打开设备 https://blog.csdn.net/zhu530548851/article/details/22070335 
    m_modbus = modbus_new_rtu(serial_port.c_str()/*serialPort*/,115200/*baud*/,'N',8/*dataBits  */,1/*stopBits*/);
    m_modbus = modbus_new_rtu(serial_port.c_str(),115200,'N',8,1);
    if (m_modbus == NULL) {
        // fprintf(stderr, "Unable to create the libmodbus context\n");
        ROS_ERROR("Unable to create the libmodbus context.\n");
        return false;
    }
    if (modbus_connect(m_modbus) == -1) {
        // fprintf(stderr, "Connection failed: %s\n",
        //         modbus_strerror(errno));
        ROS_ERROR("Connection failed: %s.\n",modbus_strerror(errno));
        modbus_free(m_modbus);
        return false;
    }
    // modbus_set_debug(m_modbus,true);
    //设置为232模式
    // modbus_rtu_set_serial_mode(m_modbus,MODBUS_RTU_RS232);
    // 设置等待时间，超过时间没连接上则报错。
    struct timeval response_timeout;
    response_timeout.tv_sec = 0;
    response_timeout.tv_usec = 1000 * 5;//5ms
    modbus_set_response_timeout(m_modbus,&response_timeout);// xx注释掉的

    int slave_addr = 1;
    modbus_set_slave(m_modbus, slave_addr);

    usleep(20*1000);//休眠20ms
    //初始化小车
    int ret;
    uint16_t state;
    ROS_INFO("sinebot init\r\n" );
    //复位
    ret = modbus_write_register(m_modbus, STATE_ADDR, (int)0);
    usleep(1000*100);
    ret = modbus_write_register(m_modbus, STATE_ADDR, (int)REQ_INIT_MOTOR);
    ROS_INFO("modbus_write_register %d  %d\r\n", REQ_INIT_MOTOR, ret);
    if(ret != 1)
    {
        ROS_INFO("sinebot init err\r\n" );
        //写入失败
        return false;
    } 
    sleep(2);//休眠2s
    ret = modbus_read_input_registers(m_modbus, STATE_ADDR, 1, (uint16_t *)&state);
    ROS_INFO("modbus_read_input_registers %d  %d\r\n", state, ret);
    if(ret == 1)
    {
        //读取成功
		//判断电机是否初始化
		if(bit_isfalse(state, STATE_INIT_MOTOR))
		{
            ROS_INFO("sinebot init  fail \r\n" );
			return false;
		}
    }
    ROS_INFO("sinebot init success\r\n" );

    return true;
}
void disconnect_modbus()
{
    //断开连接
    if(m_modbus) {
        modbus_close(m_modbus);
        modbus_free(m_modbus);
        m_modbus = NULL;
    }
}
SineBotVel get_vel()
{
    SineBotVel bot_vel;
    memset(&bot_vel,sizeof(bot_vel),0);

    if (m_modbus == NULL) {
        return bot_vel;
    }

    int ret;

    float vel[2] = {0.0};
    ret = modbus_read_input_registers(m_modbus, LINEAR_VEL_X_ADDR, 4, (uint16_t *)&vel);
    if(ret == 4)
    {
        bot_vel.linear_vel_x.vel = vel[0];
        bot_vel.angular_vel_z.vel = vel[1];
        bot_vel.linear_vel_y.vel = 0;
    }
    else
    {
        ROS_INFO("vel recive ret : %d \r\n" ,ret);
    }

    return bot_vel;
}

void set_vel(SineBotVel bot_vel)
{
    if (m_modbus == NULL) {
        return ;
    }
    float vel[2] = {0.0};
    vel[0] = bot_vel.linear_vel_x.vel;
    vel[1] = bot_vel.angular_vel_z.vel;

    int ret;
    ret = modbus_write_registers(m_modbus, LINEAR_VEL_X_ADDR, 4, (uint16_t *)vel);       
    if(ret != 4)
    {
        ROS_INFO("set_vel err ret : %d \r\n" ,ret);
    }
}