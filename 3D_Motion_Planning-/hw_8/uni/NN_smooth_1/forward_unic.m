function y=forward_hor(x)
    global M
    % layer 1
    x=M{1}*x+M{2};
    x(x<0)=0;

    % layer 2
    x=M{3}*x+M{4};
    x(x<0)=0;

    % layer 3
    x=M{5}*x+M{6};
    x(x<0)=0;

    % layer 4
    y=M{7}*x+M{8};

end
