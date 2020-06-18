function x_dot=formation_control(t,state)
    global G N weight c theta_d u_d
    x_dot = zeros(1,3*N);
    %for each node in the graph I need to compute the input and the resultant
    %kinematic
    for i=1:N
        % I retrieve the state for the current node from the congregate state
        x_i = state(3*i-2);
        y_i = state(3*i-1);
        theta_i = state(3*i);
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
            x_j = state(3*node_j-2);
            y_j = state(3*node_j-1);
            dif_x = (x_i - x_j);
            dif_y = (y_i - y_j);
            distance = [dif_x;dif_y];
            nd = norm(distance);
            kd = (1-exp(c-nd^2));
            if kd < 1e-02
                kd = 0;
            end
            u_i = u_i - (dif_x * cos(theta_i) +...
                dif_y * sin(theta_i))*kd;
            
            % theta desired equal to zero
            %omega_i = omega_i - (theta_i - kd*atan2(dif_y,dif_x));
            % theta_desired different from zero
            omega_i = omega_i - (theta_i - kd*atan2(dif_y,dif_x)) +...
                (1-kd)*theta_d;
        end
        
        % Derivatives for integration
        x_dot_i = (u_i)*cos(theta_i) + u_d*(cos(theta_d));
        y_dot_i = (u_i)*sin(theta_i) + u_d*(sin(theta_d));
        theta_dot_i = omega_i;
        x_dot(3*i-2) = x_dot_i;
        x_dot(3*i-1) = y_dot_i;
        x_dot(3*i) = theta_dot_i;
    end
    x_dot = x_dot';
end