% STEP 1.1: calculate Q_k of the k-th segment 
function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        Q_k = zeros(n_order,n_order);
        for i = 4:n_order
          for l = 4:n_order
            current_order = (i + l -7);
            coef = (factorial(i)/factorial(i-4)) * (factorial(l) / factorial(l-4)) / current_order;
            Q_k(i+1, l+1) = coef * (ts(k) ^ (i+l-7));
          end
        end
        Q = blkdiag(Q, Q_k);
    end
end

