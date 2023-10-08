function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % STEP 3.2.1 p constraint, using convex hull - the curve will be enclosed in the convex space composed of control points
    % n_order + 1 control points
    % We skip the first coef of the first segment, and the last coef of the last segment, since they have been included in position constraints
    % For simplicity, we simply acquire each control point to be in their box. 
    % However, in real life we need the last control point to be in the intesection of two boxes
    % Remember we have n_seg number of polynomials
    Aieq_p=[];
    bieq_p = [];
    % upper limit
    Aieq_p_u=[];
    bieq_p_u = [];
    Aieq_p_u = eye(n_all_poly, n_all_poly);
    Aieq_p_u = Aieq_p_u(2:end-1,:); 
    for seg=1:n_seg
        upper_lim = corridor_range(2, seg); 
        b_temp = upper_lim * ones(n_order + 1, 1); 
        bieq_p_u = [bieq_p_u; b_temp]; 
    end
    bieq_p_u = bieq_p_u(2:end-1, :);

    % lower limit
    Aieq_p_l=[];
    bieq_p_l = [];
    Aieq_p_l = -1 * eye(n_all_poly, n_all_poly);
    Aieq_p_l = Aieq_p_l(2:end-1,:);
    for seg=1:n_seg
        lower_lim = corridor_range(1, seg);
        b_temp = -lower_lim * ones(n_order + 1, 1);
        bieq_p_l = [bieq_p_l; b_temp];
    end
    bieq_p_l = bieq_p_l(2:end-1, :);

    Aieq_p = [Aieq_p_l; Aieq_p_u]; 
    bieq_p = [bieq_p_l; bieq_p_u]; 
    %#####################################################
    % STEP 3.2.2 v constraint   
    Aieq_v = [];
    bieq_v = [];

    %#####################################################
    % STEP 3.2.3 a constraint   
    Aieq_a = [];
    bieq_a = [];

    %% #####################################################
    %% combine all components to form Aieq and bieq   

    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
    Aieq = Aieq_p;
    bieq = bieq_p;
end

