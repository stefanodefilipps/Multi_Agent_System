function control = compute_control_rendezvous(t,x)
    global N G weight
    control = zeros(size(t,1),2*N);
    for k=1:size(t,1)
        for i=1:N
            % I retrieve the state for the current node from the congregate state
            x_i = x(k,3*i-2);
            y_i = x(k,3*i-1);
            theta_i = x(k,3*i);
            % Find the neighbohours
            [eid,nid] = inedges(G,i);
            u_i = 0;
            omega_i = 0;
            % Now for each neigh I need to construct the corresponding part in the
            % inout
            for j=1:size(nid)
                node_j = nid(j);
                x_j = x(k,3*node_j-2);
                y_j = x(k,3*node_j-1);
                dif_x = x_i - x_j;
                dif_y = y_i - y_j;
                u_i = u_i + weight*(dif_x * cos(theta_i) + dif_y * sin(theta_i));
                %dif_x
                %dif_y
                atan2((y_j - y_i),(x_j - x_i));
                if abs(dif_x) < 0.001 && abs(dif_y) < 0.001
                    dif_x = 0;
                    dif_y = 0;
                end
                omega_i = omega_i + weight*(theta_i - atan2(-dif_y,-dif_x));
            end
            u_i = - u_i;
            omega_i = - omega_i;
            control(k,2*i-1) = u_i;
            control(k,2*i) = omega_i;
        end
    end
end
