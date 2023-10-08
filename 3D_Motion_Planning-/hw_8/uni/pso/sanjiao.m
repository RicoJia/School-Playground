% draw a triangle, with center at (cx, cy), orientation = theta
function sanjiao(cx,cy,theta,clr)
    % x, y of 3 points 
    x = [1 -0.5 -0.5]*2.5;
    y = [0 0.5 -0.5]*2.5;
    R=[cos(theta) -sin(theta);
       sin(theta)  cos(theta)];
    % Rotate and move the three points
    pts = R*[x;y]+[cx;cy];
    patch(pts(1,:),pts(2,:),clr);
end
