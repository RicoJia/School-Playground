function global_best = pso_select(theta,omega,v_ini,p0,last_theta,last_v)
    R=[cos(theta) -sin(theta);
       sin(theta)  cos(theta)];
    %initialize the particles, and number of iters 
    N=10;
    iter=10;
    %[theta,p_end,v_theta,v_vend,best_theta,best_v,best_cost]
    P=zeros(N,7);
    global_best = zeros(1,3); %[x,y,cost?]
    global_best(1) = last_theta-theta;
    global_best(2) = last_v;
    global_best(3) = evaluate(R,omega,p0,last_theta-theta,last_theta-theta,v_ini,last_v);
    % for each particle: initialize them to: 
    for i=1:N
        P(i,1)=(rand-0.5)*1.8*pi; %(-0.9pi, 0.9pi), theta
        P(i,2)=(rand-0.5)*4+2; %(0, 4), p_end
        P(i,3)=rand-0.5; %(-0.5, 0.5)
        P(i,4)=rand-0.5;%(-0.5, 0.5)
        P(i,5)=P(i,1);
        P(i,6)=P(i,2);
        P(i,7)=inf;
    end

    for j=1:iter
        for i=1:N
            % weight of the particle, decreases as coming near the end of the iter
            w = 0.95-(0.95-0.4)/iter*j;
            if (j~=1)
                %update the particle position for the i'th particle
                k1 = 1; 
                k2 = 1; 
                P(i,3) = P(i,3) + k1 * rand * (P(i, 5) - P(i,1)) + k2 * rand * (global_best(1) - P(i, 1));
                P(i,4) = P(i,4) + k1 * rand * (P(i,6) - P(i,2)) + k2 * rand * (global_best(2) - P(i, 2));
                P(i, 1) = P(i, 1) + P(i,3); 
                P(i, 2) = P(i, 2) + P(i,4);
            end
            %evaluate the particles
            cost = evaluate(R,omega,p0,P(i,1),last_theta-theta,v_ini,P(i,2));
        
            %update the local best
            if cost < P(i,7)
                P(i,7) = cost;
                P(i,5)=P(i,1);
                P(i,6)=P(i,2);
            end
        
            %update the global best
            if cost<global_best(3)
                global_best(3)=cost;
                global_best(1)=P(i,1);
                global_best(2)=P(i,2);
            end
        end
    end

    % Not sure why need theta?
    global_best(1)=global_best(1)+theta;
end
