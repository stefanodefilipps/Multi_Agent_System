function x_dot=consensus_control(t,state)
global G N weight
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
    u_i = 0;
    omega_i = 0;
    % Now for each neigh I need to construct the corresponding part in the
    % inout
    for j=1:size(nid)
        node_j = nid(j);
        x_j = state(3*node_j-2);
        y_j = state(3*node_j-1);
        dif_x = (x_i - x_j);
        dif_y = (y_i - y_j);
        u_i = u_i + weight*(dif_x * cos(theta_i) +...
            dif_y * sin(theta_i));
        if abs(dif_x) < 0.001 && abs(dif_y) < 0.001
            dif_x = 0;
            dif_y = 0;
        end
        omega_i = omega_i + (theta_i - atan2(dif_y,dif_x));
    end
    
    u_i = - u_i;
    omega_i = - omega_i;
    x_dot_i = u_i*cos(theta_i);
    y_dot_i = u_i*sin(theta_i);
    theta_dot_i = omega_i;
    x_dot(3*i-2) = x_dot_i;
    x_dot(3*i-1) = y_dot_i;
    x_dot(3*i) = theta_dot_i;
end

x_dot = x_dot';