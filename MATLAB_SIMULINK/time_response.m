%计算时域性能指标
clc,clear

num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % 构造 G(s)
num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%构造PWMs(s)
num3 = [100 50];den3 = [1 0];PIs = tf(num3,den3)%构造PI(s)
PWM_Gs = series(PWMs,Gs)%经过PWM控制的传递函数


t = 0:0.005:10; % 设置横轴范围和步长
[y,x,t] = step(PWM_Gs,t); % 根据步长逐步响应传递函数
plot(x,y)%绘制阶跃响应 
grid on
hold on
[ymax,tp] = max(y); % 获取最大值的点的数据

r=1;
r1=0;
while y(r) < (ymax*0.9) % 过滤小于 90%的
    if y(r) > (ymax*0.1) % 过滤大于 10%的
        r1=r1+1;
    end;
    r=r+1;
end;

rise_time = r1*0.005 % 上升时间
line([rise_time,rise_time],[0,y(r1)],'linestyle',':','color','r')
text(rise_time,y(r1),'tr')

peak_time = (tp-1)*0.005; % 峰值时间 
line([peak_time,peak_time],[0,ymax],'linestyle',':','color','r') %标出峰值时间

ystable = dcgain(Gs); % 稳态值
Es = 1-ystable %稳态误差
max_overshoot = 100*(ystable-ymax)/ystable % 超调量

r2=1001; % 由 (5-0)/0.005+1 求得
while y(r2) > ystable*0.98 && y(r2) < ystable*1.02
% 从稳态倒推回去，求得首次达到并维持在稳态值正负 2%的范围的值
r2 = r2 - 1;
end
settle_time = (r2-1)*0.005 % 调节时间
line([settle_time,settle_time],[0,y(r2)],'linestyle',':','color','r')
text(settle_time,y(r2),'ts')

% num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % 构造 G(s)
% num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%构造PWMs(s)
% num3 = [100 50];den3 = [1 0];PIs = tf(num3,den3)%构造PI(s)
% PWM_Gs = series(PWMs,Gs)%经过PWM控制的传递函数
% PI_PWM_Gs = series(PIs,PWM_Gs)