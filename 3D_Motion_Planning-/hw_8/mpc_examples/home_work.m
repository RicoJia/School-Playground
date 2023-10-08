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


for t=0.2:0.2:80
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

    p_0 = pt(1, :);
    v_0 = vt(1, :);
    a_0 = at(1, :);
    %% Please follow the example in linear mpc part to fill in the code here to do the tracking
    % z-axis 
    w1 = 100;
    w2 = 1;
    w3 = 1;
    w4 = 1;
    w5 = 1e4;

    % p_0 = [x, y, z]
    for i=1:3
        [Tp, Tv, Ta, Bp, Bv, Ba] = getPredictionMatrix(K, dt, p_0(i), v_0(i), a_0(i));
        % hard constraints TODO
        % H = w1 * Tp' * Tp + w2 * Tv' * Tv + w3 * Ta' * Ta + w4 * eye(K); 
        % F = w1 * (Bp - p_0(i))' * Tp + w2 * (Bv - v_0(i))' * Tv + w3 * (Ba - a_0(i))' * Ta; 
        % A_mat = [Tv;-Tv;Ta;-Ta];
        % b = [10 * ones(20,1)-Bv;10 * ones(20,1)+Bv;ones(20,1)-Ba;ones(20,1)+Ba];
        % Soft Constraints
        H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta),w5*eye(K));
        F = [w1*(Bp-p_0(i))'*Tp+w2*(Bv - v_0(i))'*Tv+w3*(Ba - a_0(i))'*Ta zeros(1,K)];
        A_mat = [-Tv -eye(K); zeros(size(Ta)) -eye(K)];
        b = [10 * ones(20,1)+Bv; zeros(K,1)];
        
        % A_mat = [Tv zeros(K);-Tv -eye(K);Ta zeros(K); -Ta zeros(K); zeros(size(Ta)) -eye(K)];
        % b = [10 * ones(20,1)-Bv;10 * ones(20,1)+Bv; 10 * ones(20,1)-Ba; 10 * ones(20,1)+Ba; zeros(K,1)];

        J = quadprog(H, F, A_mat, b); 
        % control is the first element because we are doing this for every time.
        j = J(1);   
        p_0(i) = p_0(i) + v_0(i)*dt + 0.5*a_0(i)*dt^2 + 1/6*j*dt^3;
        v_0(i) = v_0(i) + a_0(i)*dt + 0.5*j*dt^2;
        a_0(i) = a_0(i) + j*dt; 
       % [p_0(i),v_0(i)(i),a_0(i)(i)] = forward(p_0(i),v_0(i)(i),a_0(i)(i),j(i),dt);
    end

    %% Log the states
    P = [P;p_0 pt(1,:)];
    V = [V;v_0 vt(1,:)];
    A = [A;a_0 at(1,:)];
    % P = [P;pt(1,:)];
    % V = [V;vt(1,:)];
    % A = [A;at(1,:)];
end

%% Plot the result
plot(P);
grid on;
legend('x','y','z');
figure;
plot3(P(:,1),P(:,2),P(:,3));
axis equal;
grid on;
