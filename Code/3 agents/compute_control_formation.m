function control=compute_control_formation(t,state)
    global G N weight c theta_d u_d
    control = zeros(size(t,1),2*N);
    %for each node in the graph I need to compute the input and the resultant
    %kinematic
    for k=1:size(t,1)
        for i=1:N
            % I retrieve the state for the current node from the congregate state
            x_i = state(k,3*i-2);
            y_i = state(k,3*i-1);
            theta_i = state(k,3*i);
            % Find the neighbohours
            [eid,nid] = inedges(G,i);
            % Different from zeros if we want the unicycles
            % to travel with constant velocity
            u_i = 0;
            omega_i = 0;
            % Now for each neighbor I need to construct the corresponding part in the
            % input
            for j=1:size(nid)
                node_j = nid(j);
                x_j = state(k,3*node_j-2);
                y_j = state(k,3*node_j-1);
                dif_x = (x_i - x_j);
                dif_y = (y_i - y_j);
                distance = [dif_x;dif_y];
                nd = norm(distance);
                kd = (1-exp(c-nd^2));
                u_i = u_i - (dif_x * cos(theta_i) +...
                    dif_y * sin(theta_i))*kd;
                % theta desired equal to zero
                %omega_i = omega_i - (theta_i - kd*atan2(dif_y,dif_x));
                % theta_desired different from zero
                omega_i = omega_i - (theta_i - kd*atan2(dif_y,dif_x)) +...
                    (1-kd)*theta_d;
            end
            control(k,2*i-1) = u_i;
            control(k,2*i) = omega_i;
        end
    end     
end