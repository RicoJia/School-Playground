clear all;
close all;
clc;

p_0 = [0 8 20];
v_0 = [0 0 0];
a_0 = [0 0 0];
K=20;
dt=0.2;

P=[];
V=[];
A=[];


for t=0.2:0.2:40
    %% Construct the reference signal
    for i = 1:20
        tref = t + i*0.2;
        r=0.25*tref;
        % p[x, y, z]
        pt(i,1) = r*sin(0.2*tref);
        vt(i,1) = r*cos(0.2*tref);
        at(i,1) = -r*sin(0.2*tref);
        
        pt(i,2) = r*cos(0.2*tref);
        vt(i,2) = -r*sin(0.2*tref);
        at(i,2) = -r*cos(0.2*tref);
        
        pt(i,3) = 20 - 0.5*tref;
        vt(i,3) = -0.5;
        at(i,3) = 0;
    end
    %% Please follow the example in linear mpc part to fill in the code here to do the tracking. get j at each time
    






    % p_0 = [x, y, z]
    for i=1:3
       [p_0(i),v_0(i),a_0(i)] = forward(p_0(i),v_0(i),a_0(i),j(i),dt);
    end
    
    %% Log the states
    P = [P;p_0 pt(i,:)];
    V = [V;v_0 vt(i,:)];
    A = [A;a_0 at(i,:)];
end

%% Plot the result
plot(P);
grid on;
legend('x','y','z');
figure;
plot3(P(:,1),P(:,2),P(:,3));
axis equal;
grid on;
