%频域性能指标
clc,clear
p = bodeoptions;
p.grid='on';
p.Xlim={[10,10000]}; %横坐标范围为 10~1000rad/s
p.XlimMode={'manual'};

num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % 构造 G(s)
num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%构造PWMs(s)
num3 = [100 50];den3 = [1 0];PIs = tf(num3,den3)%构造PI(s)
PWM_Gs = series(PWMs,Gs)%经过PWM控制的传递函数
PI_PWM_Gs = series(PIs,PWM_Gs)

[mag,phase,w] = bode(PI_PWM_Gs,p);
figure(1);
bode(PI_PWM_Gs,p);
hold on;
title('Bode 图');
grid on
[gm,pm,wcg,wcp]=margin(mag,phase,w); %计算相位裕度与幅值裕度
margin(PI_PWM_Gs);
fprintf('\n 幅值裕度=%2.3f\n 相位裕度=%2.3f\n',20*log10(gm),pm);
%显示幅值裕度和相位裕度

%绘制Nyquist图
figure(2);
nyquist(PI_PWM_Gs)
text(-1,0,'极点')
grid on