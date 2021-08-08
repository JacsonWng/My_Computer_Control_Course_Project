%Ƶ������ָ��
clc,clear
p = bodeoptions;
p.grid='on';
p.Xlim={[10,10000]}; %�����귶ΧΪ 10~1000rad/s
p.XlimMode={'manual'};

% num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % ���� G(s)
% num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%����PWMs(s)
% num3 = [100 50];den3 = [1 0];PIs = tf(num3,den3)%����PI(s)
% PWM_Gs = series(PWMs,Gs)%����PWM���ƵĴ��ݺ���
% PI_PWM_Gs = series(PIs,PWM_Gs)


% num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % ���� G(s)
% num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%����PWMs(s)
% num3 = [22 -18];den3 = [1 -1];PIs = tf(num3,den3)%����PI(z)
% PWM_Gs = series(PWMs,Gs)%����PWM���ƵĴ��ݺ���
% PI_PWM_Gs = series(PIs,PWM_Gs)


% num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % ���� G(s)
% num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%����PWMs(s)
% PWM_Gs = series(PWMs,Gs)%����PWM���ƵĴ��ݺ���
% num3 = [1];den3 = [1 0];G0 = tf(num3,den3)
% Ss = series(G0,PWM_Gs)
% Ts = 0.01
% Sz = c2d(Ss,Ts,'z')
% 
% z = tf('z');
% Gz = (z-1)/z
% 
% Zz = series(Gz,Sz)
% 
% z = tf('z');
% Gz2 = 1/(z-1)
% Zz2 = series(Gz2,Zz)

[mag,phase,w] = bode(PI_PWM_Gs,p);
figure(1);
bode(PI_PWM_Gs,p);
hold on;
title('Bode ͼ');
grid on
[gm,pm,wcg,wcp]=margin(mag,phase,w); %������λԣ�����ֵԣ��
margin(PI_PWM_Gs);
fprintf('\n ��ֵԣ��=%2.3f\n ��λԣ��=%2.3f\n',20*log10(gm),pm);
%��ʾ��ֵԣ�Ⱥ���λԣ��

%����Nyquistͼ
figure(2);
nyquist(PI_PWM_Gs)
text(-1,0,'����')
grid on