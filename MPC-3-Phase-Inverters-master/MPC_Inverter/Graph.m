clc
figure(1);
plot(Vc_Output,'r');
hold on
T = 0:0.01:1;
plot(Vref*sin(2*pi*T),Vref*cos(2*pi*T),'LineWidth',3);
% figure(2);
% plot(Sopt.signals.values(1,:));
figure(3);
plot(Vi_opt);
