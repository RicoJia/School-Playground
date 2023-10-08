% [R,omega,p0,last_theta-theta,last_theta-theta,v_ini,last_v]
function cost = evaluate(R,omega,p0,tgt,last_tgt,v_ini,v_end)
    %R: rotation matrix
    %omega: omega, angular speed
    %p0: initial position 2x1
    %tgt: delta_theta target
    %r: reference, point of interest
    % heatmap: cost function = dist to nearest obstacle
    global hMap

    %predict the trajectory to achieve: initial (omega, v_init) to the desired (last_v, delta_theta): npos (2, 40)
    npos = forward_unic([omega, tgt,v_ini,v_end]');
    npos = R*[npos(1:40) npos(41:80)]';
    X=[npos(1,:)]'+ p0(1);
    Y=[npos(2,:)]'+ p0(2);

    cost = 0;
    P=zeros(1,2);

    for i=1:30
        p = [X(i);Y(i)];
        
        P(1) = round(3*(p(1)+10)+1);
        P(2) = round(3*(p(2)+10)+1);
        if P(1) >0 && P(1) < 640 && P(2) >0 && P(2) < 640
            cost = cost+hMap(P(1),P(2));
        else
            cost = cost+inf;
        end
    end

    cost = cost + 8*norm(tgt-last_tgt) + 8*norm(v_end-v_ini);
end
