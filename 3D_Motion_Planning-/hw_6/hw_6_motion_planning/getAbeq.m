% TODO: se ignora el time_coefficient ts. Que Se lo anadira!
function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);

    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    % each time must be [0,1]
    Aeq_start = [];
    for i = 0:2
        new_row = get_A_coeff_single_segment(i, n_order, n_all_poly, 1, "start");
        Aeq_start = [Aeq_start; new_row]; 
    end
    beq_start = start_cond';
    
    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = [];
    for i = 0:2
        new_row = get_A_coeff_single_segment(i, n_order, n_all_poly, n_seg, "end");
        Aeq_end = [Aeq_end; new_row];
    end
    beq_end = end_cond';
    
    % No more waypoint constraints!! 
    %#####################################################
    % STEP 2.3 position continuity constrain between 2 segments
    i = 0;
    Aeq_con_p = get_continuity_constraints(i);
    beq_con_p = zeros(n_seg-1,1);

    %#####################################################
    % STEP 2.4 velocity continuity constrain between 2 segments
    i=1;
    Aeq_con_v = get_continuity_constraints(i);
    beq_con_v = zeros(n_seg-1, 1);

    %#####################################################
    % STEP 2.5 acceleration continuity constrain between 2 segments
    i=2; 
    Aeq_con_a = get_continuity_constraints(i);
    beq_con_a = zeros(n_seg-1, 1);

    function [Aeq_con] = get_continuity_constraints(i)
        Aeq_con = [];
        for seg=1:n_seg-1
            new_row_end_of_current_seg = get_A_coeff_single_segment(i, n_order, n_all_poly, seg, "end");
            new_row_start_of_next_seg = get_A_coeff_single_segment(i, n_order, n_all_poly, seg+1, "start");
            new_row =  -1 * new_row_end_of_current_seg + new_row_start_of_next_seg;
            Aeq_con = [Aeq_con; new_row];
        end
    end

    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];

    % % TODO
    % Aeq = [Aeq_start; Aeq_end];
    % beq = [beq_start; beq_end];

end

function [A_coeff] = get_A_coeff_single_segment(d_order, n_order, n_all_poly, segment, time_status)
    % Se asume q t = 0, y el numero de segment empieza de 1 y termina en n_seg 
    common_coeff = factorial(n_order) / factorial(n_order - d_order); 
    A_coeff = zeros(1, n_all_poly);
    if time_status=="start"
        starting_idx = (n_order+1) * (segment-1); 
        switch d_order
            case 0
                A_coeff(starting_idx + 1) = 1;
            case 1
                A_coeff(starting_idx + 1 : starting_idx + 2) = [-1, 1];
            case 2
                A_coeff(starting_idx + 1 : starting_idx + 3) = [1, -2, 1]; 
            otherwise 
                disp("We only supports up to 2nd order constraint (acceleration)"); 
        end
    elseif time_status=="end"
        starting_idx = (n_order + 1) * segment; %last element of the segment coeffs
        switch d_order
            case 0
                A_coeff(starting_idx) = 1;
            case 1
                A_coeff(starting_idx - 1 : starting_idx ) = [-1, 1];
            case 2
                A_coeff(starting_idx -2 : starting_idx) = [1, -2, 1];
            otherwise
                disp("We only supports up to 2nd order constraint (acceleration)");
        end
    end
    A_coeff = A_coeff * common_coeff; 
end

% Returns the Bernstein coef
function [b] = b(n, i, t)
    b = combinatorial(n, i) * t ^ (i) * (1-t)^(n-i)
end

% C_n^k
function [comb]=combinatorial(n, k)
    comb = factorial(n)/factorial(k)/factorial(n-k)
end
