%����ʱ������ָ��
clc,clear

num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % ���� G(s)
num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%����PWMs(s)
num3 = [100 50];den3 = [1 0];PIs = tf(num3,den3)%����PI(s)
PWM_Gs = series(PWMs,Gs)%����PWM���ƵĴ��ݺ���


t = 0:0.005:10; % ���ú��᷶Χ�Ͳ���
[y,x,t] = step(PWM_Gs,t); % ���ݲ�������Ӧ���ݺ���
plot(x,y)%���ƽ�Ծ��Ӧ 
grid on
hold on
[ymax,tp] = max(y); % ��ȡ���ֵ�ĵ������

r=1;
r1=0;
while y(r) < (ymax*0.9) % ����С�� 90%��
    if y(r) > (ymax*0.1) % ���˴��� 10%��
        r1=r1+1;
    end;
    r=r+1;
end;

rise_time = r1*0.005 % ����ʱ��
line([rise_time,rise_time],[0,y(r1)],'linestyle',':','color','r')
text(rise_time,y(r1),'tr')

peak_time = (tp-1)*0.005; % ��ֵʱ�� 
line([peak_time,peak_time],[0,ymax],'linestyle',':','color','r') %�����ֵʱ��

ystable = dcgain(Gs); % ��ֵ̬
Es = 1-ystable %��̬���
max_overshoot = 100*(ystable-ymax)/ystable % ������

r2=1001; % �� (5-0)/0.005+1 ���
while y(r2) > ystable*0.98 && y(r2) < ystable*1.02
% ����̬���ƻ�ȥ������״δﵽ��ά������ֵ̬���� 2%�ķ�Χ��ֵ
r2 = r2 - 1;
end
settle_time = (r2-1)*0.005 % ����ʱ��
line([settle_time,settle_time],[0,y(r2)],'linestyle',':','color','r')
text(settle_time,y(r2),'ts')

% num1 = [0.01];den1 = [0.005 0.06 0.1];Gs = tf(num1,den1) % ���� G(s)
% num2 = [2.4];den2 = [0.002 1];PWMs = tf(num2,den2)%����PWMs(s)
% num3 = [100 50];den3 = [1 0];PIs = tf(num3,den3)%����PI(s)
% PWM_Gs = series(PWMs,Gs)%����PWM���ƵĴ��ݺ���
% PI_PWM_Gs = series(PIs,PWM_Gs)