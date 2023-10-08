function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    % n_order = 7

    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    beq_start = start_cond
    % STEP 2.1: write expression of Aeq_start and beq_start
    t = 0;
    for k = 0:3
      for i = k:n_order %each element in the polynomial, starting from k.
        Aeq_start(k+1, i+1) = factorial(i)/factorial(i-k) * (t^(i-k)); 
      end
    end
    
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    beq_end = end_cond
    % STEP 2.2: write expression of Aeq_end and beq_end
    t = ts(end);
    for k = 0:3
      for i = k:n_order %each element in the polynomial, starting from k.
        Aeq_end(k+1, i+1) = factorial(i)/factorial(i-k) * (t^(i-k)); 
      end
    end
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for j=1:n_seg-1
      Aeq_wp(j, j*8+1) = 1
      beq_wp(j) = waypoints(j+1)
    end
    
    %#####################################################
    % position continuity constrain between each 2 segments
    % Aeq_con_p = zeros(n_seg-1, n_all_poly);
    % beq_con_p = zeros(n_seg-1, 1
    k = 0;    % k is the order of derivative
    Aeq_con_p = get_Aeq_con(n_seg, n_order, ts, n_all_poly, k);
    beq_con_p = zeros(n_seg-1, 1);

    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    k = 1;
    Aeq_con_v = get_Aeq_con(n_seg, n_order, ts, n_all_poly, k);
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v


    %#####################################################
    % acceleration continuity constrain between each 2 segments
    k = 2; 
    Aeq_con_a = get_Aeq_con(n_seg, n_order, ts, n_all_poly, k);
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a

    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    k = 3; 
    Aeq_con_j = get_Aeq_con(n_seg, n_order, ts, n_all_poly, k);
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j

    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end

function [Aeq_con] = get_Aeq_con(n_seg, n_order, ts, n_all_poly, k)
  Aeq_con = zeros(n_seg-1, n_all_poly);
  beq_con = zeros(n_seg-1, 1);
  for j=1:n_seg-1
    t = ts(j);
    for i=k:n_order
      idx = (j-1) * n_order + i+1;
      Aeq_con(j, idx) = factorial(i)/factorial(i-k) * (t^(i-k));
    end

    t = 0;
    for i=k:n_order %order of the current coef
      idx = (j) * (n_order+1) + i+1;  %8 elements to the right, we subtract
      Aeq_con(j, idx) = -factorial(i)/factorial(i-k) * (t^(i-k));
    end
  end
end
