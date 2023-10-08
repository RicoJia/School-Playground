
clc;clear;close all;
% TODO
% path = ginput() * 100.0;
% path = [[1.0, 2.0]; [4.0, 5.0]] 
path = [[1.0, 2.0]; [4.0, 5.0]; [10.0, 8.0]; [12, 9]];

n_order       = 7;% order of poly, we need pvaj, minimum snap needs up to jerK? 
n_seg         = size(path,1)-1;% number of segments
n_poly_perseg = (n_order+1); % number of perseg coef

ts = allocate_t_equal_dist(n_seg, path);
% ts = allocate_t_dist(n_seg, path)

poly_coef_x = MinimumSnapQPSolver(path(:, 1), ts, n_seg, n_order);
poly_coef_y = MinimumSnapQPSolver(path(:, 2), ts, n_seg, n_order);

% display the trajectory
X_n = [];
Y_n = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis
    Pxi = flipud(poly_coef_x(i*8+1:i*8+8))
    Pyi = flipud(poly_coef_y(i*8+1:i*8+8))
    for t = 0:tstep:ts(i+1)
        X_n(k)  = polyval(Pxi, t);
        Y_n(k)  = polyval(Pyi, t);
        k = k + 1;
    end
end

plot(X_n, Y_n , 'Color', [0 1.0 0], 'LineWidth', 2);
hold on
scatter(path(1:size(path, 1), 1), path(1:size(path, 1), 2));



function [ts]=allocate_t_dist(n_seg, path)
  % calculate time distribution in proportion to distance between 2 points
  ts = zeros(n_seg, 1);
  dist     = zeros(n_seg, 1);
  dist_sum = 0;
  T        = 25;
  t_sum    = 0;

  for i = 1:n_seg
      dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
      dist_sum = dist_sum+dist(i);
  end
  for i = 1:n_seg-1
      ts(i) = dist(i)/dist_sum*T;
      t_sum = t_sum+ts(i);
  end
  ts(n_seg) = T - t_sum;
end

function [ts]=allocate_t_equal_dist(n_seg, path)
  ts = zeros(n_seg, 1);

  % time distribution you can simply set all time distribution as 1
  for i = 1:n_seg
      ts(i) = 1.0;
  end
end

function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0].';
    end_cond   = [waypoints(end), 0, 0, 0].';
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts)
    %#####################################################
    % STEP 2: compute Aeq and beq
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)

    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end

