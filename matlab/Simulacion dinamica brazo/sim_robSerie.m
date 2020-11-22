clc,clear
robot = importrobot('..\Simulacion dinamica brazo\Modelos urdf brazo\Robot\urdf\Robot.urdf');
%robot = importrobot('..\Simulacion dinamica brazo\Modelos urdf brazo\Brazo\urdf\Brazo.urdf');
robot.DataFormat = 'column';
robot.Gravity=[0 0 -9.81];
%%
% Simulación de cinemática inversa
t = (0:0.01 :10)'; % Time

count = length(t);
% center = [0 0 -1];
% radius = 0.2;
% theta = t*(8*pi/t(end));
% points = center + radius*[cos(theta) sin(theta) 0.07*t];

points = [0.4-0.08*t 0.5-0.01*t.^2 -ones(size(t))*0.8];

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess

%% Planificador de trayectorias interpolador cartesiano
waypoints = [0.011265502281722 0.032831860285015   -1.078889510983337;
    0.4 0.4 -0.8;
    0.4 -0.4  -0.8;
    -0.4 -0.4  -0.8;
    -0.4 0.4  -0.8];
waypoints = waypoints';
timepoints = [0 0.5 1 1.5 2];
t = 0:0.1:2;
q =quinticpolytraj(waypoints,timepoints,t);

posx = timeseries(q(1,:),t);
posy = timeseries(q(2,:),t);
posz = timeseries(q(3,:),t);

%% Planificador de trayectorias interpolador articular
% waypoints = [-0.0330 0.0338   -1.0770;
%     0.4 0.4 -0.8;
%     0 -0.3  -0.5];
% t1 = trvec2tform(waypoints(1,:));
% t2 = trvec2tform(waypoints(2,:));
% t3 = trvec2tform(waypoints(3,:));
% qSol1 = ik(endEffector,t1,weights,q0);
% qSol2 = ik(endEffector,t2,weights,qSol1);
% 
% waypoints = [q0 qSol1 qSol2];
% timepoints = [0 1 2];
% t = 0:0.1:2;
% q =quinticpolytraj(waypoints,timepoints,t);
% q1 = timeseries(q(1,:),t);
% q2 = timeseries(q(2,:),t);
% q3 = timeseries(q(3,:),t);
% q4 = timeseries(q(4,:),t);

%% Simulación
out = sim('robot_serie',t(end));

%% Mostrar resultados
salto = round(length(out.jointData)/60);
figure(1)
show(robot,out.jointData(1,:)','Frames','off');
axis([-1.5 1.5 -1.5 1.5 -1.5 0.3])
hold on
pause 
plot3(q(1,:),q(2,:),q(3,:))
fps = 30;
r = rateControl(fps);
for i = 1:salto:length(out.jointData)
    t_act = out.tout(i,1);
    titulo = 'Tiempo actual: ';
    titulo = strcat(titulo,num2str(t_act));
    show(robot,out.jointData(i,:)','PreservePlot',false,'Frames','off');
    axis([-1.5 1.5 -1.5 1.5 -1.3 0.3])
    drawnow
    title(titulo)
    waitfor(r);
end
hold off
pause
close all
figure(1)
plot(out.torque1.Time,out.torque1.Data)
title('Torque nesesario en la articulación 1')
xlabel('t(s)')
ylabel('T (N m)')
grid on
figure(2)
plot(out.torque2.Time,out.torque2.Data)
title('Torque nesesario en la articulación 2')
xlabel('t(s)')
ylabel('T (N m)')
grid on
figure(3)
plot(out.torque3.Time,out.torque3.Data)
title('Torque nesesario en la articulación 3')
xlabel('t(s)')
ylabel('T (N m)')
grid on
figure(4)
plot(out.torque4.Time,out.torque4.Data)
title('Torque nesesario en la articulación 4')
xlabel('t(s)')
ylabel('T (N m)')
grid on
figure(5)
plot(out.tout,out.torque1.Data.*out.velocidades(:,1))
title('Potencia nesesaria en la articulación 1')
xlabel('t(s)')
ylabel('P (W)')
grid on
figure(6)
plot(out.tout,out.torque2.Data.*out.velocidades(:,2))
title('Potencia nesesaria en la articulación 2')
xlabel('t(s)')
ylabel('P (W)')
grid on
figure(7)
plot(out.tout,out.torque3.Data.*out.velocidades(:,3))
title('Potencia nesesaria en la articulación 3')
xlabel('t(s)')
ylabel('P (W)')
grid on
figure(8)
plot(out.tout,out.torque4.Data.*out.velocidades(:,4))
title('Potencia nesesaria en la articulación 4')
xlabel('t(s)')
ylabel('P (W)')
grid on
