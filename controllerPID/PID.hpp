#include <cmath>
#include <algorithm>
#pragma once

namespace Controller{
    class PID
    {
        public:
        bool IntegralSeparation = false;//积分分离选项 
        PID():kp(0),ki(0),kd(0),max_iout(0),limit_output(0){};
        PID(float p,float i, float d,float lim_out,float lim_iout,float obs):kp(p),ki(i),kd(d),current(obs),max_iout(lim_iout),limit_output(lim_out){};
        void calculate(){
            error = target - current;
            pout = kp*error;
            iout += ki*error;
            iout = std::clamp(iout,-max_iout,max_iout);//简单的积分限幅
            dout = kd*(error - pre_error);
            output = pout + iout + dout;
            output = std::clamp(output, -limit_output, limit_output);
            pre_error = error;
        }
        void setObserver(const float &obs){
            current = obs;
        }
        float update(float tar){
            target = tar;
            calculate();
            return output;
        }
        private:
        float kp,ki,kd;
        float current = 0.f,target = 0.f;
        float error = 0.f,pre_error = 0.f;
        float pout = 0.f,iout = 0.f,dout = 0.f,max_iout = 0.f;
        float output = 0.f,limit_output = 0.f;
    };
};